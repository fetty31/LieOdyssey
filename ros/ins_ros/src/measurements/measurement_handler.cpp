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
}

// -----------------------------------------------------------------------------
// Push
// -----------------------------------------------------------------------------

template <typename T>
void push(const T& measurement)
{
    std::lock_guard<std::mutex> lock(mutex_);

    QueuedMeasurement queued{Measurement{measurement}};

    insertMeasurement(queued);

    if constexpr (std::is_same_v<T, iESEKF::IMUmeas>)
    {
        insertSorted(imu_history_, measurement);
        pruneHistory(measurement.stamp);
    }

    enforceCapacity();
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

template <typename T>
std::optional<T> MeasurementHandler::popOfType()
{
    std::lock_guard<std::mutex> lock(mutex_);

    for (auto it = measurement_queue_.begin();
         it != measurement_queue_.end();
         ++it)
    {
        if (std::holds_alternative<T>(it->measurement))
        {
            T measurement = std::get<T>(it->measurement);
            measurement_queue_.erase(it);
            return measurement;
        }
    }

    return std::nullopt;
}

template <typename T>
std::optional<T> MeasurementHandler::peekOfType() const
{
  for (const auto& measurement : measurement_queue_)
  {
    if (std::holds_alternative<T>(measurement.measurement))
      return std::get<T>(measurement.measurement);
  }

  return std::nullopt;
}

template <typename T>
std::vector<T> MeasurementHandler::peekNOfType(std::size_t n) const
{
    std::lock_guard<std::mutex> lock(mutex_);

    std::vector<T> result;
    result.reserve(n);

    for (const auto& queued : measurement_queue_)
    {
        if (const auto* measurement = std::get_if<T>(&queued.measurement))
        {
            result.push_back(*measurement);

            if (result.size() >= n)
                break;
        }
    }

    return result;
}

void MeasurementHandler::insertMeasurement(
    const QueuedMeasurement& measurement)
{
  const double stamp = stamp(measurement);

  if (measurement_queue_.empty() ||
      stamp >= stamp(measurement_queue_.back()))
  {
    measurement_queue_.push_back(measurement);
    return;
  }

  auto it = std::upper_bound(
      measurement_queue_.begin(),
      measurement_queue_.end(),
      stamp,
      [](double t, const QueuedMeasurement& m) {
        return t < stamp(m);
      });

  measurement_queue_.insert(it, measurement);
}

bool MeasurementHandler::hasMeasurements() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return !measurement_queue_.empty();
}

double MeasurementHandler::lateststamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (measurement_queue_.empty())
    return -1.0;

  return stamp(measurement_queue_.back());
}

double MeasurementHandler::nextstamp() const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (measurement_queue_.empty())
    return -1.0;

  return stamp(measurement_queue_.front());
}

std::size_t MeasurementHandler::measurementsQueued() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return measurement_queue_.size();
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

// -----------------------------------------------------------------------------
// Pruning
// -----------------------------------------------------------------------------

void MeasurementHandler::pruneOlderThan(double t_min)
{
  std::lock_guard<std::mutex> lock(mutex_);

  while (!measurement_queue_.empty() &&
         stamp(measurement_queue_.front()) < t_min)
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

template <typename T>
void MeasurementHandler::insertSorted(
    std::deque<T>& buffer,
    const T& sample)
{
  if (buffer.empty() ||
      sample.stamp >= buffer.back().stamp)
  {
    buffer.push_back(sample);
    return;
  }

  auto it = std::upper_bound(
      buffer.begin(),
      buffer.end(),
      sample.stamp,
      [](double stamp, const T& element) {
        return stamp < element.stamp;
      });

  buffer.insert(it, sample);
}

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
}

double MeasurementHandler::stamp(const QueuedMeasurement& measurement)
{
  return std::visit(
      [](const auto& m) -> double {
        return m.stamp;
      },
      measurement.measurement);
}

}  // namespace ins_ros::measurements