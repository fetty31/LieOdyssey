#include "ins_ros/measurements/measurement_handler.hpp"

#include <algorithm>
#include <cmath>

namespace ins_ros::measurements {

MeasurementHandler::MeasurementHandler() : options_() {}
MeasurementHandler::MeasurementHandler(const Options& options) : options_(options) {}

void MeasurementHandler::setOptions(const Options& options) {
  std::lock_guard<std::mutex> lock(mutex_);
  options_ = options;
  enforceCapacity();
}

void MeasurementHandler::clear() {
  std::lock_guard<std::mutex> lock(mutex_);
  imu_queue_.clear();
  imu_history_.clear();
  gps_buffer_.clear();
  odom_buffer_.clear();
  wheel_buffer_.clear();
  mag_buffer_.clear();
  baro_buffer_.clear();
  yaw_buffer_.clear();
  state_history_.clear();
}

// --- Push ---

void MeasurementHandler::pushImu(const iESEKF::IMUmeas& imu) {
  std::lock_guard<std::mutex> lock(mutex_);
  // Keep buffers sorted by stamp; callbacks are normally monotonic,
  // but handle small out-of-order arrivals gracefully.
  auto insert_sorted = [](auto& buffer, const auto& sample, double stamp) {
    if (buffer.empty() || stamp >= buffer.back().stamp) {
      buffer.push_back(sample);
      return;
    }
    auto it = std::upper_bound(buffer.begin(), buffer.end(), stamp,
                               [](double s, const auto& e) { return s < e.stamp; });
    buffer.insert(it, sample);
  };
  insert_sorted(imu_queue_, imu, imu.stamp);
  insert_sorted(imu_history_, imu, imu.stamp);
  pruneHistory(imu.stamp);
  enforceCapacity();
}

void MeasurementHandler::pushGps(const StampedGps& gps) {
  std::lock_guard<std::mutex> lock(mutex_);
  gps_buffer_.push_back(gps);
  enforceCapacity();
}

void MeasurementHandler::pushOdom(const StampedOdom& odom) {
  std::lock_guard<std::mutex> lock(mutex_);
  odom_buffer_.push_back(odom);
  enforceCapacity();
}

void MeasurementHandler::pushWheel(const StampedWheel& wheel) {
  std::lock_guard<std::mutex> lock(mutex_);
  wheel_buffer_.push_back(wheel);
  enforceCapacity();
}

void MeasurementHandler::pushMag(const StampedMag& mag) {
  std::lock_guard<std::mutex> lock(mutex_);
  mag_buffer_.push_back(mag);
  enforceCapacity();
}

void MeasurementHandler::pushBaro(const StampedBaro& baro) {
  std::lock_guard<std::mutex> lock(mutex_);
  baro_buffer_.push_back(baro);
  enforceCapacity();
}

void MeasurementHandler::pushYaw(const StampedYaw& yaw) {
  std::lock_guard<std::mutex> lock(mutex_);
  yaw_buffer_.push_back(yaw);
  enforceCapacity();
}

// --- IMU queries ---

bool MeasurementHandler::hasImu() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !imu_queue_.empty();
}

double MeasurementHandler::latestImuStamp() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (imu_queue_.empty()) return -1.0;
  return imu_queue_.back().stamp;
}

std::vector<iESEKF::IMUmeas> MeasurementHandler::drainImuUpTo(double t_query) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<iESEKF::IMUmeas> out;
  while (!imu_queue_.empty() && imu_queue_.front().stamp <= t_query) {
    out.push_back(imu_queue_.front());
    imu_queue_.pop_front();
  }
  return out;
}

std::vector<iESEKF::IMUmeas> MeasurementHandler::imuBetween(double t0, double t1) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<iESEKF::IMUmeas> out;
  for (const auto& imu : imu_history_) {
    if (imu.stamp > t0 && imu.stamp <= t1) out.push_back(imu);
  }
  return out;
}

// --- Generic helpers ---

template <typename T>
std::optional<T> MeasurementHandler::takeLatestAtOrBefore(std::deque<T>& buffer,
                                                          double t_query) {
  std::optional<T> best;
  while (!buffer.empty() && buffer.front().stamp <= t_query) {
    best = buffer.front();
    buffer.pop_front();
  }
  return best;
}

template <typename T, typename StampFn>
std::optional<T> MeasurementHandler::takeClosestTo(std::deque<T>& buffer, double t_query,
                                                   double tolerance,
                                                   double future_tolerance,
                                                   StampFn stamp_of) {
  // Find index of closest sample within [t_query - tolerance, t_query + future_tolerance].
  int best_idx = -1;
  double best_dt = std::numeric_limits<double>::max();
  const int n = static_cast<int>(buffer.size());
  for (int i = 0; i < n; ++i) {
    const double s = stamp_of(buffer[static_cast<std::size_t>(i)]);
    if (s < t_query - tolerance) continue;  // too old, will be discarded below
    if (s > t_query + future_tolerance) break;  // buffers are time-ordered
    const double dt = std::abs(s - t_query);
    if (dt < best_dt) {
      best_dt = dt;
      best_idx = i;
    }
  }
  // Discard everything strictly older than the search window.
  while (!buffer.empty() && stamp_of(buffer.front()) < t_query - tolerance) {
    buffer.pop_front();
    if (best_idx > 0) --best_idx;
  }
  if (best_idx < 0) return std::nullopt;
  // Consume up to and including the chosen sample.
  T out = buffer[static_cast<std::size_t>(best_idx)];
  buffer.erase(buffer.begin(), buffer.begin() + best_idx + 1);
  return out;
}

// --- Aiding queries ---

std::optional<StampedGps> MeasurementHandler::takeGpsAtOrBefore(double t_query) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeLatestAtOrBefore(gps_buffer_, t_query);
}

std::optional<StampedOdom> MeasurementHandler::takeOdomAtOrBefore(double t_query) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeLatestAtOrBefore(odom_buffer_, t_query);
}

std::optional<StampedWheel> MeasurementHandler::takeWheelAtOrBefore(double t_query) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeLatestAtOrBefore(wheel_buffer_, t_query);
}

std::optional<StampedMag> MeasurementHandler::takeMagAtOrBefore(double t_query) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeLatestAtOrBefore(mag_buffer_, t_query);
}

std::optional<StampedBaro> MeasurementHandler::takeBaroAtOrBefore(double t_query) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeLatestAtOrBefore(baro_buffer_, t_query);
}

std::optional<StampedYaw> MeasurementHandler::takeYawAtOrBefore(double t_query) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeLatestAtOrBefore(yaw_buffer_, t_query);
}

std::optional<StampedGps> MeasurementHandler::takeSyncGps(double t_query, double tolerance,
                                                          double future_tolerance) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeClosestTo(gps_buffer_, t_query, tolerance, future_tolerance,
                       [](const StampedGps& e) { return e.stamp; });
}

std::optional<StampedOdom> MeasurementHandler::takeSyncOdom(double t_query, double tolerance,
                                                            double future_tolerance) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeClosestTo(odom_buffer_, t_query, tolerance, future_tolerance,
                       [](const StampedOdom& e) { return e.stamp; });
}

std::optional<StampedWheel> MeasurementHandler::takeSyncWheel(double t_query, double tolerance,
                                                              double future_tolerance) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeClosestTo(wheel_buffer_, t_query, tolerance, future_tolerance,
                       [](const StampedWheel& e) { return e.stamp; });
}

std::optional<StampedMag> MeasurementHandler::takeSyncMag(double t_query, double tolerance,
                                                          double future_tolerance) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeClosestTo(mag_buffer_, t_query, tolerance, future_tolerance,
                       [](const StampedMag& e) { return e.stamp; });
}

std::optional<StampedBaro> MeasurementHandler::takeSyncBaro(double t_query, double tolerance,
                                                            double future_tolerance) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeClosestTo(baro_buffer_, t_query, tolerance, future_tolerance,
                       [](const StampedBaro& e) { return e.stamp; });
}

std::optional<StampedYaw> MeasurementHandler::takeSyncYaw(double t_query, double tolerance,
                                                          double future_tolerance) {
  std::lock_guard<std::mutex> lock(mutex_);
  return takeClosestTo(yaw_buffer_, t_query, tolerance, future_tolerance,
                       [](const StampedYaw& e) { return e.stamp; });
}

std::optional<StampedGps> MeasurementHandler::peekLatestGps() const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (gps_buffer_.empty()) return std::nullopt;
  return gps_buffer_.back();
}

std::vector<StampedGps> MeasurementHandler::peekNewestGps(std::size_t n) const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<StampedGps> out;
  if (gps_buffer_.empty() || n == 0) return out;
  const std::size_t count = std::min(n, gps_buffer_.size());
  out.reserve(count);
  for (std::size_t i = gps_buffer_.size() - count; i < gps_buffer_.size(); ++i) {
    out.push_back(gps_buffer_[i]);
  }
  return out;
}

// --- State history ---

void MeasurementHandler::pushStateSnapshot(double stamp, const iESEKF::Group& state,
                                           const iESEKF::MatDoF& cov) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!state_history_.empty() && stamp <= state_history_.back().stamp) {
    // Re-propagation after rewind may rewrite recent history; replace the tail.
    while (!state_history_.empty() && state_history_.back().stamp >= stamp) {
      state_history_.pop_back();
    }
  }
  state_history_.push_back(StateSnapshot{stamp, state, cov});
  while (state_history_.size() > options_.state_capacity) state_history_.pop_front();
}

std::optional<StateSnapshot> MeasurementHandler::snapshotAt(double t_query) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_history_.empty()) return std::nullopt;
  std::optional<StateSnapshot> best;
  for (const auto& snap : state_history_) {
    if (snap.stamp <= t_query) {
      best = snap;
    } else {
      break;
    }
  }
  // If every snapshot is newer than the query (should not happen for delayed
  // measurements within the history window), fall back to the oldest one.
  if (!best) best = state_history_.front();
  return best;
}

void MeasurementHandler::truncateSnapshotsAfter(double t_query) {
  std::lock_guard<std::mutex> lock(mutex_);
  while (!state_history_.empty() && state_history_.back().stamp > t_query) {
    state_history_.pop_back();
  }
}

void MeasurementHandler::pruneOlderThan(double t_min) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto prune = [t_min](auto& buffer) {
    while (!buffer.empty() && buffer.front().stamp < t_min) buffer.pop_front();
  };
  prune(gps_buffer_);
  prune(odom_buffer_);
  prune(wheel_buffer_);
  prune(mag_buffer_);
  prune(baro_buffer_);
  prune(yaw_buffer_);
  prune(imu_queue_);
  prune(imu_history_);
  prune(state_history_);
}

// --- Introspection ---

std::size_t MeasurementHandler::imuQueued() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return imu_queue_.size();
}

std::size_t MeasurementHandler::gpsQueued() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return gps_buffer_.size();
}

std::size_t MeasurementHandler::odomQueued() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return odom_buffer_.size();
}

std::size_t MeasurementHandler::wheelQueued() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return wheel_buffer_.size();
}

std::size_t MeasurementHandler::yawQueued() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return yaw_buffer_.size();
}

std::size_t MeasurementHandler::baroQueued() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return baro_buffer_.size();
}

std::size_t MeasurementHandler::magQueued() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mag_buffer_.size();
}

bool MeasurementHandler::needsRewind(double meas_stamp, double filter_time,
                                     double latency_threshold) {
  return (filter_time - meas_stamp) > latency_threshold;
}

// --- Private ---

void MeasurementHandler::enforceCapacity() {
  while (imu_queue_.size() > options_.imu_capacity) imu_queue_.pop_front();
  while (imu_history_.size() > options_.imu_capacity) imu_history_.pop_front();
  while (gps_buffer_.size() > options_.aiding_capacity) gps_buffer_.pop_front();
  while (odom_buffer_.size() > options_.aiding_capacity) odom_buffer_.pop_front();
  while (wheel_buffer_.size() > options_.aiding_capacity) wheel_buffer_.pop_front();
  while (mag_buffer_.size() > options_.aiding_capacity) mag_buffer_.pop_front();
  while (baro_buffer_.size() > options_.aiding_capacity) baro_buffer_.pop_front();
  while (yaw_buffer_.size() > options_.aiding_capacity) yaw_buffer_.pop_front();
  while (state_history_.size() > options_.state_capacity) state_history_.pop_front();
}

void MeasurementHandler::pruneHistory(double t_newest) {
  const double t_min = t_newest - options_.history_window_s;
  while (!imu_history_.empty() && imu_history_.front().stamp < t_min) {
    imu_history_.pop_front();
  }
  while (!state_history_.empty() && state_history_.front().stamp < t_min) {
    state_history_.pop_front();
  }
}

}  // namespace ins_ros::measurements
