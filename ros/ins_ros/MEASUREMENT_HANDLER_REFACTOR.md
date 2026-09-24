# Measurement Handler + Fixed-Frequency Timer Refactor

**Date:** 2026-09-09
**Package:** `ros/ins_ros`
**Build status:** `colcon build --packages-select ins_ros` passes.

## Goal

Decouple ROS callbacks from filter execution:

- Callbacks only convert ROS messages and push stamped measurements into a central handler.
- A fixed-frequency timer owns all `filter_.predict()` / `filter_.update()` calls.
- Delayed measurements (notably GPS) are handled by rewinding to a past filter
  snapshot and re-propagating buffered IMU (past/future state recovery).

## New files

- `include/ins_ros/measurements/stamped_types.hpp`
  - `StampedImu`, `StampedGps`, `StampedOdom`, `StampedWheel`, `StampedMag`,
    `StampedBaro`, `StampedYaw`: each carries a `stamp` in ROS header seconds plus
    value and noise (`R` / `R_inv`).
  - `StateSnapshot{stamp, Group state, MatDoF covariance}` for rewind history.
- `include/ins_ros/measurements/measurement_handler.hpp`
- `src/measurements/measurement_handler.cpp`
  - Thread-safe (single mutex) central buffer.
  - IMU kept twice: a consumable queue (`drainImuUpTo`) plus non-destructive
    history (`imuBetween`) so rewind can re-propagate.
  - Out-of-order IMU inserts are sorted on push; history pruned by
    `history_window_s`; capacities enforced on push.

## MeasurementHandler API

| Method | Semantics |
|---|---|
| `pushImu/Gps/Odom/Wheel/Mag/Baro/Yaw()` | Called only from ROS callbacks. |
| `hasImu()`, `latestImuStamp()` | Timer target selection. |
| `drainImuUpTo(t)` | Pops and returns all queued IMU with `stamp <= t` (sorted). |
| `imuBetween(t0, t1)` | Non-destructive read of IMU history in `(t0, t1]` for re-propagation. |
| `take*AtOrBefore(t)` | Latest measurement with `stamp <= t`; consumes it and all older ones. Used for GPS (rewind-capable). |
| `takeSync*(t, tolerance, future_tolerance)` | Closest measurement to `t` in `[t - tolerance, t + future_tolerance]`; consumes everything up to and including it. Used for odom/wheel/mag/baro/yaw. |
| `pushStateSnapshot(stamp, state, cov)` | Records filter belief; rewrites the tail if `stamp` goes backwards (re-propagation after rewind). |
| `snapshotAt(t)` | Closest snapshot with `stamp <= t` (falls back to oldest if all are newer). |
| `truncateSnapshotsAfter(t)` | Drops snapshots newer than `t` after a rewind. |
| `pruneOlderThan(t_min)` | Bounds all buffers. |
| `needsRewind(meas_stamp, filter_time, threshold)` | `true` when `filter_time - meas_stamp > threshold`. |

## Estimator changes (`estimator_node.hpp` / `estimator_node.cpp`)

- Removed `boost::circular_buffer` members (`state_buffer_`, `imu_buffer_`) and
  wall-clock `t0_system_`; added `MeasurementHandler meas_handler_`,
  `rclcpp::TimerBase::SharedPtr estimation_timer_`, `filter_time_` /
  `filter_time_initialized_` (sensor-stamp time base).
- Callbacks are thin:
  - `imu_callback`: dt from sensor stamps, transform to base_link (lever-arm
    correction), feed stationary initializer, `pushImu`.
  - `gps_callback`: fix check, ENU origin init, LLA→ENU, debug publish, feed
    yaw initializers, build `StampedGps` (`R` from message or fixed noise),
    `pushGps`. Continuous-heading output is pushed as `StampedYaw` instead of
    updating the filter inline.
  - `odom_callback`: extrinsics init, LIO-world→ENU (or LIO-body→base when no
    GPS), build `StampedOdom`, `pushOdom` + debug publish.
  - `wheel_odom/mag/baro_callback`: build stamped structs, push.
- New `estimation_timer_callback()` at `filter.rate`:
  1. Gate on ENU origin (if GPS enabled) and orientation init; drains IMU while
     waiting so queues stay bounded. On fresh init, anchors `filter_time_` to
     newest IMU and seeds snapshot history.
  2. `process_imu_up_to(latest_imu_stamp)`: sequential `predict` per IMU
     (`0 < dt < 0.1` guard), `group_to_state` with sensor stamp, snapshot push.
  3. `process_gps_at()`: `takeGpsAtOrBefore`; drop if `delay > gps.max_age`;
     if `delay > rewind_threshold` → `apply_gps_with_rewind()` (restore
     snapshot, GPS update at old stamp, truncate, re-predict IMUs up to
     `filter_time`); else direct update. Snapshot refreshed afterwards.
  4. `process_odom/wheel/mag/baro/yaw_at()`: `takeSync*` + direct update +
     snapshot refresh.
  5. Single `publish_odom()` / `publish_pose()` / `broadcast_tf()` per tick;
     `pruneOlderThan(filter_time - history_window)`.
- `state_.time` is now the sensor stamp (was wall-clock since configure);
  `from_ins_to_ros` stamps output headers from `state.time`.
- `publish_yaw_debug()` extracted from the old inline GPS yaw block.
- Timer lifecycle: created in `on_activate` via `setup_timer()`, destroyed in
  `on_deactivate` / `on_cleanup` / `on_shutdown` / `on_error`.

## Parameters (added to `config/ins_ros.yaml` and `config/kitti.yaml`)

```yaml
filter:
  rate: 100.0            # estimation timer frequency [Hz]
  history_window: 5.0    # IMU/snapshot history kept for rewind [s]
  buffer:
    imu_capacity: 2000
    aiding_capacity: 200
sync:
  gps:
    rewind_threshold: 0.05  # delay above this triggers rewind + re-propagation [s]
    max_age: 2.0            # older GPS is dropped as stale [s]
  odom:        {tolerance: 0.05}
  wheel_odom:  {tolerance: 0.05}
  mag:         {tolerance: 0.05}
  baro:        {tolerance: 0.05}
  yaw:         {tolerance: 0.10}
  future_tolerance: 0.02
```

## Build / integration

- `CMakeLists.txt`: `src/measurements/measurement_handler.cpp` added to
  `${PROJECT_NAME}_core`.
- Verified with `colcon build --packages-select ins_ros
  --cmake-args -DCMAKE_BUILD_TYPE=Release` (only pre-existing Eigen/SSE
  warnings from `yaw_handler.hpp` remain).
- Single-threaded executor still used; handler mutex makes a future
  multi-threaded executor safe. Callbacks read `state_` (bias/debug) while the
  timer writes it — safe under the current single-threaded executor.

## Known limitations / follow-ups

1. After a GPS rewind only IMU is re-propagated; other aiding measurements
   inside `(gps_stamp, filter_time]` are not re-applied (current code saves
   `X_cur`/`P_cur` placeholders for that future work).
2. `imu_callback` still owns `last_imu_stamp_` / `previous_omega_base_` for dt
   and lever-arm correction — could move into the handler later.
3. Mag/baro noise is still fixed in code (params declared but unused) — same as
   before the refactor.
4. Consider re-validating KITTI/on-robot runs: output stamp semantic changed
   from wall-clock to sensor-stamp.

## Update 2026-09-09 — init anchoring on newest buffered GPS + velocity seed

**Problem:** heading initialization needs motion (`distance_threshold` ≈ 2 m),
so at `initialize_orientation()` time the robot is no longer at the ENU origin
(first fix), but the filter was anchored at `p = -R * lever_arm`, i.e. antenna
at origin.

**Fix (`estimator_node.cpp::initialize_orientation`):**
- Peeks (non-destructive) the two newest buffered GPS fixes via new handler
  methods `peekLatestGps()` / `peekNewestGps(n)` — the initializer stays
  heading-only, no fix duplication there.
- Anchors `p_body = p_antenna_newest - R * lever_arm` (same for
  `initial_enu_base_`, used for LIO alignment).
- Seeds `state_.v` from fix differencing (horizontal plane only; GPS altitude
  too noisy), gated on `dt > 0` and speed ≤ 30 m/s, else zero velocity.
- Extrapolates the anchored position to the latest IMU stamp
  (`p += v * (t_ref - t_gps)`, GPS typically lags IMU by up to ~100–200 ms)
  and sets `state_.time = t_ref`, matching what the timer anchors
  `filter_time_` to right after init.
- Buffered fixes are intentionally *not* consumed, so the next timer tick
  still applies them as the first GPS update.

## Update 2026-09-09 — receive-time stamping (`timing.use_receive_stamp`)

**Problem:** sync/rewind assumes all stamps share one clock. If a sensor (e.g.
GPS) stamps in its own unsynchronized clock, cross-sensor sync silently breaks.

**Fix:** new `timing.use_receive_stamp` param (default `false`, in both yaml
configs). When true, `INSEstimator::sensor_stamp()` returns
`get_clock()->now()` and every input — IMU (incl. the filter time base and
`dt`), GPS, odom, wheel, mag, baro — is stamped at callback arrival instead of
trusting `msg.header.stamp`. Outputs already use the filter time base, so they
stay consistent automatically. Trade-off: IMU `dt` becomes inter-arrival time
with scheduling jitter, but on a common clock, which is strictly better than a
wrong clock.
