#pragma once

#include <deque>
#include <mutex>
#include <optional>
#include <variant>
#include <vector>
#include <cstddef>

#include "ins_ros/measurements/stamped_types.hpp"

namespace ins_ros::measurements {

// -----------------------------------------------------------------------------
// Measurement types
// -----------------------------------------------------------------------------

using Measurement = std::variant<
    iESEKF::IMUmeas,
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

  template <typename T>
  void push(const T& measurement);

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
  std::optional<T> peekOfType() const;

  /**
   * @brief Return a vector of n unprocessed measurements of a specific
   * type without consuming it. The returned vector is ordered chronologically
   * (oldest first). An empty vector is returned if no measurements are found.
   * If there are less than n measurements the vector is returned filled with 
   * the number of measurements present, that is n is a limit size condition
   */
  template <typename T>
  std::vector<T> peekNOfType(std::size_t n) const;

  /**
   * @brief Remove and return the oldest unprocessed measurement.
   */
  std::optional<QueuedMeasurement> pop();

  /**
   * @brief Remove and return the oldest unprocessed measurement 
   * of a specific type.
   */
  template <typename T>
  std::optional<T> popOfType();

  /**
   * @brief Check whether there are unprocessed measurements.
   */
  bool hasMeasurements() const;

  /**
   * @brief Timestamp of the oldest queued measurement.
   *
   * Returns -1.0 if the queue is empty.
   */
  double nextMeasurementStamp() const;

  /**
   * @brief Timestamp of the newest queued measurement.
   *
   * Returns -1.0 if the queue is empty.
   */
  double latestMeasurementStamp() const;

  /**
   * @brief Number of measurements waiting for processing.
   */
  std::size_t measurementsQueued() const;

  // ---------------------------------------------------------------------------
  // IMU history
  // ---------------------------------------------------------------------------

  /**
   * @brief Get historical IMU measurements in (t0, t1].
   *
   * Does not consume the IMU history.
   */
  std::vector<iESEKF::IMUmeas>
  imuBetween(double t0, double t1) const;

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
  static double stamp(
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
      const T& sample);

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
   * @brief Single live processing queue containing ALL sensor measurements.
   *
   * Always maintained in chronological order.
   *
   * Example:
   *
   *   IMU   1.000
   *   IMU   1.005
   *   ODOM  1.007
   *   IMU   1.010
   *   GPS   1.015
   *   YAW   1.020
   */
  std::deque<QueuedMeasurement> measurement_queue_;

  /**
   * @brief Historical IMU measurements retained for rewind/repropagation.
   *
   * This is intentionally separate from measurement_queue_. Once an IMU
   * measurement has been processed and removed from the live queue, it must
   * still be available to repropagate the filter after a delayed update.
   */
  std::deque<iESEKF::IMUmeas> imu_history_;

  /**
   * @brief Historical filter states/covariances used for rewind.
   */
  std::deque<StateSnapshot> state_history_;

  /**
   * @brief Protects all queues and configuration.
   *
   * Mutable so const query methods can lock the mutex.
   */
  mutable std::mutex mutex_;
};

}  // namespace ins_ros::measurements

class MeasurementHandler
{
public:

    template <typename T>
    void push(const T& measurement);

    std::optional<QueuedMeasurement> peek() const;

    std::optional<QueuedMeasurement> pop();

    bool hasMeasurements() const;

    double nextMeasurementStamp() const;

    double latestMeasurementStamp() const;

    std::size_t measurementsQueued() const;

    // Historical data
    std::vector<iESEKF::IMUmeas>
    imuBetween(double t0, double t1) const;

    void pushStateSnapshot(
        double stamp,
        const iESEKF::Group& state,
        const iESEKF::MatDoF& covariance);

    std::optional<StateSnapshot>
    snapshotAt(double t) const;

private:

    template <typename T>
    void insertSorted(std::deque<T>& buffer, const T& sample);

    void insertMeasurement(const QueuedMeasurement& measurement);

    static double stamp(const QueuedMeasurement& measurement);

    std::deque<QueuedMeasurement> measurement_queue_;

    std::deque<iESEKF::IMUmeas> imu_history_;

    std::deque<StateSnapshot> state_history_;

    Options options_;

    mutable std::mutex mutex_;
};