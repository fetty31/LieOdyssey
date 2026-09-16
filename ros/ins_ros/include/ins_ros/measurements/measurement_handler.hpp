#pragma once

// Project includes
#include "ins_ros/measurements/stamped_types.hpp"

#include <deque>
#include <mutex>
#include <optional>
#include <vector>

namespace ins_ros::measurements {

/**
 * @brief Centralized buffering / synchronization of all filter inputs.
 *
 * Ownership model:
 *  - Callbacks only call push*() (non-blocking, thread-safe).
 *  - A fixed-frequency timer in the estimator drains IMU up to the
 *    current filter time and pulls synchronized aiding measurements
 *    via take*() helpers.
 *  - Delayed measurements (e.g. GPS) are handled by rewinding the
 *    filter to a StateSnapshot at the measurement stamp and
 *    re-propagating buffered IMU (see state history API).
 */
class MeasurementHandler {
 public:
  struct Options {
    std::size_t imu_capacity{2000};
    std::size_t aiding_capacity{200};
    std::size_t state_capacity{2000};
    double history_window_s{5.0};  // how long IMU/state history is kept for rewind
  };

  explicit MeasurementHandler();
  explicit MeasurementHandler(const Options& options);

  // --- Configuration ---
  void setOptions(const Options& options);
  void clear();

  // --- Push (called from ROS callbacks) ---
  void pushImu(const iESEKF::IMUmeas& imu);
  void pushGps(const StampedGps& gps);
  void pushOdom(const StampedOdom& odom);
  void pushWheel(const StampedWheel& wheel);
  void pushMag(const StampedMag& mag);
  void pushBaro(const StampedBaro& baro);
  void pushYaw(const StampedYaw& yaw);

  // --- IMU queries (called from timer) ---
  bool hasImu() const;
  double latestImuStamp() const;
  /// Destructively pop all queued IMU with stamp <= t_query (sorted).
  std::vector<iESEKF::IMUmeas> drainImuUpTo(double t_query);
  /// Non-destructive read of IMU history in (t0, t1] for re-propagation.
  std::vector<iESEKF::IMUmeas> imuBetween(double t0, double t1) const;

  // --- Synchronized aiding queries ---
  /// Latest measurement with stamp <= t_query. Consumes it and all older ones.
  std::optional<StampedGps> takeGpsAtOrBefore(double t_query);
  std::optional<StampedOdom> takeOdomAtOrBefore(double t_query);
  std::optional<StampedWheel> takeWheelAtOrBefore(double t_query);
  std::optional<StampedMag> takeMagAtOrBefore(double t_query);
  std::optional<StampedBaro> takeBaroAtOrBefore(double t_query);
  std::optional<StampedYaw> takeYawAtOrBefore(double t_query);

  /**
   * @brief Closest measurement to t_query within tolerance.
   *
   * Only looks at measurements with stamp <= t_query + future_tolerance
   * (to allow for small timestamp jitter). Consumes everything up to and
   * including the returned measurement so it cannot be reused.
   * Returns nullopt if nothing is within tolerance.
   */
  std::optional<StampedGps> takeSyncGps(double t_query, double tolerance,
                                        double future_tolerance = 0.02);
  std::optional<StampedOdom> takeSyncOdom(double t_query, double tolerance,
                                          double future_tolerance = 0.02);
  std::optional<StampedWheel> takeSyncWheel(double t_query, double tolerance,
                                            double future_tolerance = 0.02);
  std::optional<StampedMag> takeSyncMag(double t_query, double tolerance,
                                        double future_tolerance = 0.02);
  std::optional<StampedBaro> takeSyncBaro(double t_query, double tolerance,
                                          double future_tolerance = 0.02);
  std::optional<StampedYaw> takeSyncYaw(double t_query, double tolerance,
                                        double future_tolerance = 0.02);

  // --- Non-destructive peek (buffer contents are preserved) ---

  /**
   * @brief Get newest measurement from buffer
   *
   * Gets last measurement from buffer (if not empty)
   */
  std::optional<StampedGps> peekLatestGps() const;
  std::optional<StampedOdom> peekLatestOdom() const;
  std::optional<StampedWheel> peekLatestWheel() const;
  std::optional<StampedMag> peekLatestMag() const;
  std::optional<StampedBaro> peekLatestBaro() const;
  std::optional<StampedYaw> peekLatestYaw() const;

  /**
   * @brief Get up to n newest buffered messages in chronological order (oldest first).
   *
   * Gets last n measurements from buffer taking into account a time tolerance
   * between consecutive samples. In case the tolerance is not respected, returns
   * an empty vector.
   */
  std::vector<StampedGps> peekNewestGps(std::size_t n, double tolerance=0.1) const;
  std::vector<StampedOdom> peekNewestOdom(std::size_t n, double tolerance=0.1) const;
  std::vector<StampedWheel> peekNewestWheel(std::size_t n, double tolerance=0.1) const;
  std::vector<StampedMag> peekNewestMag(std::size_t n, double tolerance=0.1) const;
  std::vector<StampedBaro> peekNewestBaro(std::size_t n, double tolerance=0.1) const;
  std::vector<StampedYaw> peekNewestYaw(std::size_t n, double tolerance=0.1) const;

  /**
   * @brief Closest measurement to t_query within tolerance without consuming.
   *
   * Only looks at measurements with stamp <= t_query + future_tolerance
   * (to allow for small timestamp jitter).
   * Returns nullopt if nothing is within tolerance.
   */
  std::optional<StampedGps> peekSyncGps(double t_query, double tolerance,
                                        double future_tolerance = 0.02) const;
  std::optional<StampedOdom> peekSyncOdom(double t_query, double tolerance,
                                          double future_tolerance = 0.02) const;
  std::optional<StampedWheel> peekSyncWheel(double t_query, double tolerance,
                                            double future_tolerance = 0.02) const;
  std::optional<StampedMag> peekSyncMag(double t_query, double tolerance,
                                        double future_tolerance = 0.02) const;
  std::optional<StampedBaro> peekSyncBaro(double t_query, double tolerance,
                                          double future_tolerance = 0.02) const;
  std::optional<StampedYaw> peekSyncYaw(double t_query, double tolerance,
                                        double future_tolerance = 0.02) const;

  // --- State history (for past/future recovery on delayed measurements) ---
  void pushStateSnapshot(double stamp, const iESEKF::Group& state,
                         const iESEKF::MatDoF& cov);

  /// Closest snapshot with stamp <= t_query. Nullopt if history is empty/too new.
  std::optional<StateSnapshot> snapshotAt(double t_query) const;

  /// Drop snapshots newer than t_query (used after rewind + re-propagation).
  void truncateSnapshotsAfter(double t_query);

  //// Prune (remove) all buffer's data older than t_min
  void pruneOlderThan(double t_min);

  // --- Introspection ---
  std::size_t imuQueued() const;
  std::size_t gpsQueued() const;
  std::size_t odomQueued() const;
  std::size_t wheelQueued() const;
  std::size_t yawQueued() const;
  std::size_t baroQueued() const;
  std::size_t magQueued() const;

  static bool needsRewind(double meas_stamp, double filter_time,
                          double latency_threshold);

//  private:
 public:
  template <typename T>
  static std::optional<T> takeLatestAtOrBefore(std::deque<T>& buffer, double t_query);

  template <typename T, typename StampFn>
  static std::optional<T> takeClosestTo(std::deque<T>& buffer, double t_query,
                                        double tolerance, double future_tolerance,
                                        StampFn stamp_of);

  template <typename T, typename StampFn>
  static std::optional<T> peekClosestTo(const std::deque<T>& buffer, double t_query,
                                        double tolerance, double future_tolerance,
                                        StampFn stamp_of);

  template <typename T, typename StampFn>
  static std::vector<T> peekNewest(const std::deque<T>& buffer, std::size_t n,
                                  double tolerance,
                                  StampFn stamp_of);
  
  template <typename T>
  static std::optional<T> peekLatest(const std::deque<T>& buffer);

  void enforceCapacity();
  void pruneHistory(double t_newest);

  mutable std::mutex mutex_;
  Options options_;

  // Pending IMU queue (consumed by drain) + full history (for re-propagation).
  std::deque<iESEKF::IMUmeas> imu_queue_;
  std::deque<iESEKF::IMUmeas> imu_history_;

  std::deque<StampedGps> gps_buffer_;
  std::deque<StampedOdom> odom_buffer_;
  std::deque<StampedWheel> wheel_buffer_;
  std::deque<StampedMag> mag_buffer_;
  std::deque<StampedBaro> baro_buffer_;
  std::deque<StampedYaw> yaw_buffer_;

  // Monotonic filter snapshots keyed by stamp.
  std::deque<StateSnapshot> state_history_;
};

}  // namespace ins_ros::measurements
