#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <variant>
#include <vector>
#include <cstddef>
#include <iomanip>

#include "ins_ros/measurements/stamped_types.hpp"

namespace ins_ros::measurements {

// -----------------------------------------------------------------------------
// Measurement types
// -----------------------------------------------------------------------------

using Measurement = std::variant<
    StampedGps,
    StampedOdom,
    StampedWheel,
    StampedMag,
    StampedBaro,
    StampedYaw>;

/**
 * @brief Measurement stored in the global chronological processing queue.
 *
 * The timestamp is intentionally NOT duplicated here. Every measurement type
 * already contains its own `stamp` member.
 */
struct QueuedMeasurement
{
  Measurement measurement;
};

// -----------------------------------------------------------------------------
// MeasurementHandler
// -----------------------------------------------------------------------------

class MeasurementHandler
{
public:

  struct Options
  {
    // Maximum number of measurements in the global processing queue.
    std::size_t measurement_capacity = 10000;

    // Maximum number of IMU samples kept for repropagation.
    std::size_t imu_capacity = 20000;

    // Maximum number of state snapshots.
    std::size_t state_capacity = 10000;

    // Maximum number of processed measurements saved for rewind
    std::size_t processed_capacity = 500;

    // Amount of history retained for rewind/repropagation.
    double history_window_s = 10.0;
  };

  MeasurementHandler();

  explicit MeasurementHandler(const Options& options);

  void setOptions(const Options& options);

  // ---------------------------------------------------------------------------
  // Lifecycle
  // ---------------------------------------------------------------------------

  void clear();

  // ---------------------------------------------------------------------------
  // Push measurements
  // ---------------------------------------------------------------------------

  /**
   * @brief Add measurement to the buffer.
   */
  template <typename T>
  void push(const T& measurement)
  {
      std::lock_guard<std::mutex> lock(mutex_);

      if constexpr (std::is_same_v<T, iESEKF::IMUmeas>)
      {
          insertSorted(imu_history_, measurement);
          pruneHistory(measurement.stamp);
      }
      else
      {
          QueuedMeasurement queued{Measurement{measurement}};
          insertMeasurement(queued);
      }

      enforceCapacity();
  }

  // ---------------------------------------------------------------------------
  // Global chronological measurement queue
  // ---------------------------------------------------------------------------

  /**
   * @brief Return the oldest unprocessed measurement without consuming it.
   */
  std::optional<QueuedMeasurement> peek() const;

  /**
   * @brief Return the oldest unprocessed measurement of a specific
   * type without consuming it.
   */
  template <typename T>
  std::optional<T> peekOfType() const
  {
    for (const auto& measurement : measurement_queue_)
    {
    if (std::holds_alternative<T>(measurement.measurement))
        return std::get<T>(measurement.measurement);
    }

    return std::nullopt;
  }

  /**
   * @brief Return a vector of n unprocessed measurements of a specific
   * type without consuming it. The returned vector is ordered chronologically
   * (oldest first). An empty vector is returned if no measurements are found.
   * If there are less than n measurements the vector is returned filled with 
   * the number of measurements present, that is n is a limit size condition
   */
  template <typename T>
  std::vector<T> peekNOfType(std::size_t n) const
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

  /**
   * @brief Remove and return the oldest unprocessed measurement.
   */
  std::optional<QueuedMeasurement> pop();

  /**
   * @brief Remove and return the oldest unprocessed measurement 
   * of a specific type.
   */
  template <typename T>
  std::optional<T> popOfType()
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

  /**
   * @brief Check whether there are unprocessed measurements.
   */
  bool hasMeasurements() const;

  /**
   * @brief Number of measurements waiting for processing.
   */
  std::size_t queuedCount() const;

  /**
   * @brief Number of measurements waiting for processing
   * of a specific type.
   */
  template <typename T>
  std::size_t queuedCountOfType() const
  {
    std::lock_guard<std::mutex> lock(mutex_);

    return static_cast<std::size_t>(
      std::count_if(
          measurement_queue_.begin(),
          measurement_queue_.end(),
          [](const QueuedMeasurement& queued) {
            return std::holds_alternative<T>(queued.measurement);
          }));
  }

  // --------------------------------------------------------------------------- 
  // Processed measurements
  // ---------------------------------------------------------------------------

  /**
  * @brief Store a measurement after it has been processed.
  */
  void markProcessed(const QueuedMeasurement& measurement);

  /**
  * @brief Return all processed measurements in (t0, t1].
  */
  std::vector<QueuedMeasurement> processedBetween(double t0, double t1) const;  

  /**
  * @brief Remove processed measurements older than t_min.
  */
  void pruneProcessedHistory(double t_min);

    /**
   * @brief Remove processed measurements after t.
   *
   * Measurements with stamp > t are removed (used for removing tail before OOSM handling).
   */
  void eraseProcessedAfter(double t);

  // ---------------------------------------------------------------------------
  // Odom utils
  // ---------------------------------------------------------------------------

  /**
   * @brief Get last processed Odometry measurement 
   * (used in relative odometry measurement)
   *
   */
  std::optional<StampedOdom> peekLatestProcessedOdom() const;

  /**
   * @brief Get processed Odometry measurement at time t
   * (used in relative odometry measurement)
   *
   */
  std::optional<StampedOdom> peekProcessedOdomAt(double t) const;

  // ---------------------------------------------------------------------------
  // IMU history
  // ---------------------------------------------------------------------------

  /**
   * @brief Get historical IMU measurements in (t0, t1].
   *
   * Does not consume the IMU history.
   */
  std::vector<iESEKF::IMUmeas> imuBetween(double t0, double t1) const;

  /**
   * @brief Interpolate IMU data to given t
   *
   * Does not consume the IMU history.
   * Requires IMU_before < t < IMU_after for interpolation
   */
  std::optional<iESEKF::IMUmeas> interpolateImuAt(double t) const;

  /**
   * @brief Get closest IMU measurement to t (always older than t)
   *
   * Does not consume the IMU history.
   */
  std::optional<iESEKF::IMUmeas> latestImuAtOrBefore(double t) const;

  // ---------------------------------------------------------------------------
  // State history
  // ---------------------------------------------------------------------------

  /**
   * @brief Store a state/covariance snapshot.
   *
   * Snapshots must normally be inserted chronologically. If a snapshot with
   * an existing timestamp is inserted after a rewind, the old tail is removed.
   */
  void pushStateSnapshot(
      double stamp,
      const iESEKF::Group& state,
      const iESEKF::MatDoF& cov);

  /**
   * @brief Get the latest snapshot at or before t_query.
   *
   * If all snapshots are newer than t_query, the oldest snapshot is returned.
   */
  std::optional<StateSnapshot>
  snapshotAt(double t_query) const;

  /**
   * @brief Remove history states after rewind
   *
   * States with stamp > t are removed.
   */
  void eraseStateHistoryAfter(double t);

  /**
   * @brief Return a copy of the full state/covariance history.
   *
   * The copy is taken while holding the handler mutex so the caller
   * can release it before doing any (slow) I/O with the returned data.
   * The returned snapshots are ordered chronologically (oldest first).
   */
  std::vector<StateSnapshot> stateHistory() const;

  // ---------------------------------------------------------------------------
  // History management
  // ---------------------------------------------------------------------------

  /**
   * @brief Remove old queued/history measurements.
   *
   * Measurements with stamp < t_min are removed.
   */
  void pruneOlderThan(double t_min);

  /**
  * @brief Get the timestamp from a queued measurement.
  *
  * Implemented using std::visit over the Measurement variant.
  */
  static double getStamp(
      const QueuedMeasurement& measurement);

private:

  // ---------------------------------------------------------------------------
  // Generic queue helpers
  // ---------------------------------------------------------------------------

  /**
   * @brief Insert a measurement into the global queue while preserving
   * chronological ordering.
   */
  void insertMeasurement(
      const QueuedMeasurement& measurement);

  /**
   * @brief Insert a stamped element into a chronologically ordered deque.
   */
  template <typename T>
  static void insertSorted(
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

  // ---------------------------------------------------------------------------
  // Capacity/history
  // ---------------------------------------------------------------------------

  void enforceCapacity();

  void pruneHistory(double t_newest);

private:
  // ---------------------------------------------------------------------------
  // Data
  // ---------------------------------------------------------------------------

  Options options_;

  /**
  * @brief Chronological queue of pending non-IMU measurements.
  *
  * IMU samples are handled separately through imu_history_ because they
  * form the propagation timeline and are retained independently for
  * repropagation.
  */
  std::deque<QueuedMeasurement> measurement_queue_;

  /**
  * @brief Raw IMU timeline retained for propagation and repropagation.
  *
  * IMU samples are not consumed when the filter propagates. They remain
  * available for OOSM replay within the history window.
  */
  std::deque<iESEKF::IMUmeas> imu_history_;

  /**
   * @brief Historical filter states/covariances used for rewind.
   */
  std::deque<StateSnapshot> state_history_;

  /**
  * @brief Non-IMU measurements that have been incorporated into the filter.
  *
  * Used to reconstruct the sequence of aiding updates after an OOSM rewind.
  */
  std::deque<QueuedMeasurement> processed_history_;

  /**
   * @brief Protects all queues and configuration.
   *
   * Mutable so const query methods can lock the mutex.
   */
  mutable std::mutex mutex_;
};

}  // namespace ins_ros::measurements