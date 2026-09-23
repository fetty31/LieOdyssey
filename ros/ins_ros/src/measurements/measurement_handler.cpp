#include "ins_ros/measurements/measurement_handler.hpp"

namespace ins_ros::measurements {

MeasurementHandler::MeasurementHandler()
    : options_()
{
}

MeasurementHandler::MeasurementHandler(const Options& options)
    : options_(options)
{
}

void MeasurementHandler::setOptions(const Options& options)
{
  std::lock_guard<std::mutex> lock(mutex_);
  options_ = options;
  enforceCapacity();
}

void MeasurementHandler::clear()
{
  std::lock_guard<std::mutex> lock(mutex_);

  measurement_queue_.clear();
  imu_history_.clear();
  state_history_.clear();
  processed_history_.clear();
}

// -----------------------------------------------------------------------------
// Generic measurement queue
// -----------------------------------------------------------------------------

std::optional<QueuedMeasurement> 
MeasurementHandler::peek() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (measurement_queue_.empty())
        return std::nullopt;

    return measurement_queue_.front();
}

std::optional<QueuedMeasurement> 
MeasurementHandler::pop()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (measurement_queue_.empty())
        return std::nullopt;

    QueuedMeasurement measurement =
        std::move(measurement_queue_.front());

    measurement_queue_.pop_front();

    return measurement;
}

void MeasurementHandler::insertMeasurement(
    const QueuedMeasurement& measurement)
{
  const double stamp = getStamp(measurement);

  if (measurement_queue_.empty() ||
      stamp >= getStamp(measurement_queue_.back()))
  {
    measurement_queue_.push_back(measurement);
    return;
  }

  auto it = std::upper_bound(
      measurement_queue_.begin(),
      measurement_queue_.end(),
      stamp,
      [](double t, const QueuedMeasurement& m) {
        return t < getStamp(m);
      });

  measurement_queue_.insert(it, measurement);
}

bool MeasurementHandler::hasMeasurements() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !measurement_queue_.empty();
}

std::size_t MeasurementHandler::queuedCount() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return measurement_queue_.size();
}

void MeasurementHandler::markProcessed(
    const QueuedMeasurement& measurement)
{
    std::lock_guard<std::mutex> lock(mutex_);

    // Normally measurements arrive chronologically, so this is just
    // a push_back. OOSM insertion is handled by sorted insertion.
    const double stamp = getStamp(measurement);

    auto it = std::upper_bound(
        processed_history_.begin(),
        processed_history_.end(),
        stamp,
        [](double t, const QueuedMeasurement& q)
        {
            return t < getStamp(q);
        });

    processed_history_.insert(it, measurement);
}

std::vector<QueuedMeasurement>
MeasurementHandler::processedBetween(
    double t0,
    double t1) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<QueuedMeasurement> result;

    for (const auto& measurement : processed_history_)
    {
        const double t = getStamp(measurement);

        if (t <= t0)
            continue;

        if (t > t1)
            break;

        result.push_back(measurement);
    }

    return result;
}

void MeasurementHandler::pruneProcessedHistory(double t_min)
{
    std::lock_guard<std::mutex> lock(mutex_);

    while (!processed_history_.empty() &&
           getStamp(processed_history_.front()) < t_min)
    {
        processed_history_.pop_front();
    }
}

// -----------------------------------------------------------------------------
// Odom utils
// -----------------------------------------------------------------------------

std::optional<StampedOdom>
MeasurementHandler::peekLatestProcessedOdom() const
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto it = processed_history_.rbegin();
         it != processed_history_.rend();
         ++it)
    {
        if (std::holds_alternative<StampedOdom>(it->measurement))
            return std::get<StampedOdom>(it->measurement);
    }

    return std::nullopt;
}

// -----------------------------------------------------------------------------
// IMU history
// -----------------------------------------------------------------------------

std::vector<iESEKF::IMUmeas>
MeasurementHandler::imuBetween(double t0, double t1) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  std::vector<iESEKF::IMUmeas> out;

  for (const auto& imu : imu_history_)
  {
    if (imu.stamp > t0 && imu.stamp <= t1)
      out.push_back(imu);

    if (imu.stamp > t1)
      break;
  }

  return out;
}

std::optional<iESEKF::IMUmeas>
MeasurementHandler::interpolateImuAt(double t) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (imu_history_.size() < 2)
        return std::nullopt;

    // Find the first sample strictly after t.
    auto it = std::upper_bound(
        imu_history_.begin(),
        imu_history_.end(),
        t,
        [](double stamp, const iESEKF::IMUmeas& imu)
        {
            return stamp < imu.stamp;
        });

    // Need one sample before and one after t.
    if (it == imu_history_.begin() ||
        it == imu_history_.end())
    {
        return std::nullopt;
    }

    const auto& next = *it;
    const auto& prev = *(it - 1);

    const double dt_samples =
        next.stamp - prev.stamp;

    if (dt_samples <= 0.0)
        return std::nullopt;

    const double alpha =
        (t - prev.stamp) / dt_samples;

    iESEKF::IMUmeas interpolated;

    interpolated.stamp = t;

    interpolated.accel =
        (1.0 - alpha) * prev.accel +
        alpha * next.accel;

    interpolated.gyro =
        (1.0 - alpha) * prev.gyro +
        alpha * next.gyro;

    // dt is NOT the interpolation interval between the two
    // measurements. It is the propagation interval represented
    // by this synthesized sample. The caller will set this.
    interpolated.dt = 0.0;

    return interpolated;
}

// -----------------------------------------------------------------------------
// State history
// -----------------------------------------------------------------------------

void MeasurementHandler::pushStateSnapshot(
    double stamp,
    const iESEKF::Group& state,
    const iESEKF::MatDoF& cov)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!state_history_.empty() &&
        stamp <= state_history_.back().stamp)
    {
        while (!state_history_.empty() &&
                state_history_.back().stamp >= stamp)
        {
            state_history_.pop_back();
        }
    }

    state_history_.push_back(
        StateSnapshot{stamp, state, cov});

    pruneHistory(stamp);
    enforceCapacity();
}

std::optional<StateSnapshot>
MeasurementHandler::snapshotAt(double t_query) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (state_history_.empty())
        return std::nullopt;

    std::optional<StateSnapshot> best;

    for (const auto& snap : state_history_)
    {
    if (snap.stamp <= t_query)
        best = snap;
    else
        break;
    }

    if (!best)
        best = state_history_.front();

    return best;
}

void MeasurementHandler::eraseStateHistoryAfter(double t)
{
    std::lock_guard<std::mutex> lock(mutex_);

    while (!state_history_.empty() &&
           state_history_.back().stamp > t)
    {
        state_history_.pop_back();
    }
}

// -----------------------------------------------------------------------------
// Pruning
// -----------------------------------------------------------------------------

void MeasurementHandler::pruneOlderThan(double t_min)
{
  std::lock_guard<std::mutex> lock(mutex_);

  while (!measurement_queue_.empty() &&
         getStamp(measurement_queue_.front()) < t_min)
  {
    measurement_queue_.pop_front();
  }

  while (!imu_history_.empty() &&
         imu_history_.front().stamp < t_min)
  {
    imu_history_.pop_front();
  }

  while (!state_history_.empty() &&
         state_history_.front().stamp < t_min)
  {
    state_history_.pop_front();
  }
}

// -----------------------------------------------------------------------------
// Private helpers
// -----------------------------------------------------------------------------

void MeasurementHandler::enforceCapacity()
{
  while (measurement_queue_.size() >
         options_.measurement_capacity)
  {
    measurement_queue_.pop_front();
  }

  while (imu_history_.size() >
         options_.imu_capacity)
  {
    imu_history_.pop_front();
  }

  while (state_history_.size() >
         options_.state_capacity)
  {
    state_history_.pop_front();
  }

  while (processed_history_.size() >
        options_.processed_capacity)
  {
    processed_history_.pop_front();
  }
}

void MeasurementHandler::pruneHistory(double t_newest)
{
  const double t_min =
      t_newest - options_.history_window_s;

  while (!imu_history_.empty() &&
         imu_history_.front().stamp < t_min)
  {
    imu_history_.pop_front();
  }

  while (!state_history_.empty() &&
         state_history_.front().stamp < t_min)
  {
    state_history_.pop_front();
  }

  while (!processed_history_.empty() && 
        getStamp(processed_history_.front()) < t_min)
  {
    processed_history_.pop_front();
  }
}

double MeasurementHandler::getStamp(const QueuedMeasurement& measurement)
{
  return std::visit(
      [](const auto& m) -> double {
        return m.stamp;
      },
      measurement.measurement);
}

}  // namespace ins_ros::measurements