# ins_ros: Inertial Navigation System

A full Inertial Navigation System (INS) estimator for ROS 2, built on the **Iterative Error-State Extended Kalman Filter (iESEKF)** defined on Lie groups (specifically **SGal(3)**). It fuses IMU, GPS, wheel odometry, 3D odometry, magnetometer (to-do), and barometer (to-do) measurements to produce robust pose and velocity estimates.

## Features

- **iESEKF on SGal(3)**: State estimation on the Lie group SGal(3) (position + velocity + orientation + time) with right-plus perturbation
- **Multi-Sensor Fusion**: Supports IMU, GPS, wheel odometry, 3D odometry (LIO/VIO), magnetometer, and barometer
- **Lifecycle Node**: Full ROS 2 lifecycle node with configure, activate, deactivate, cleanup, and shutdown transitions
- **OOSM Handling**: Out-of-sequence measurement processing with fixed-lag smoothing
- **ENU Frame Initialization**: Automatic GPS-based ENU datum setup
- **Online Calibration**: Automatic IMU orientation and bias initialization from stationary measurements
- **Continuous GPS Yaw**: Optional continuous yaw refinement from GPS trajectory
- **TF Broadcasting**: Automatic frame broadcasting (world → body)
- **Configurable via YAML**: Extensive parameterization for all sensors and filter options

## Prerequisites

### System Requirements
- Ubuntu 20.04 or later
- ROS 2 Humble or later
- C++17 compatible compiler

### Dependencies
- **lie_odyssey_cpp**: Lie group-based library for state-space estimation
- **GeographicLib**: For GPS LLA to ENU conversion
- **Eigen3**: Linear algebra library

### ROS 2 Dependencies
- `rclcpp`, `rclcpp_lifecycle`: ROS C++ client library
- `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `visualization_msgs`: Message types
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`: Transform library and ROS integration

## Installation

### From Source

1. **Clone the repository** into your ROS 2 workspace:
    ```bash
    cd ~/colcon_ws/src
    # LieOdyssey should already be cloned
    ```

2. **Install dependencies**:
    ```bash
    rosdep install --from-paths src --ignore-src -r -y
    ```

3. **Build the workspace**:
    ```bash
    cd ~/colcon_ws
    colcon build --packages-select ins_ros
    ```

4. **Source the setup script**:
    ```bash
    source install/setup.bash
    ```

## Usage

### Launch the Node

Run the INS estimator node:

```bash
ros2 launch ins_ros ins_ros.launch.py
```

#### Launch Arguments

- `config`: Path to YAML configuration file (default: `config/ins_ros.yaml`)
- `rviz`: Enable RViz visualization (default: `False`)
- `use_sim_time`: Use simulation time (default: `False`)

Example with RViz:
```bash
ros2 launch ins_ros ins_ros.launch.py rviz:=True
```

Example with custom config:
```bash
ros2 launch ins_ros ins_ros.launch.py config:=/path/to/custom_config.yaml
```

### Pre-configured Datasets

Three example configurations are provided:

- **`config/ins_ros.yaml`**: General purpose configuration
- **`config/kitti.yaml`**: Calibrated for the [KITTI](https://www.cvlibs.net/datasets/kitti/) dataset
- **`config/ona.yaml`**: Calibrated for the [ONA](https://www.vaivelogistics.com/) robot

### Launch configurations

```bash
# General configuration
ros2 launch ins_ros ins_ros.launch.py

# ONA robot configuration
ros2 launch ins_ros ona.launch.py

# KITTI dataset configuration
ros2 launch ins_ros kitti.launch.py
```

## Configuration

The package uses YAML configuration files located in the `config/` directory. Key configuration parameters:

```yaml
ins_ros_node:
  ros__parameters:
    frames:
      world: odom              # World/global frame
      body: base_link          # Robot body frame

    tf:
      publish: true            # Publish TF transforms

    timing:
      use_receive_stamp: false # Stamp measurements at receive time

    filter:
      iterations:
        max: 5                 # iESEKF max iterations
        tolerance: 1e-5
      rate: 100.0              # Estimation frequency [Hz]
      history_window: 5.0      # OOSM smoothing window [s]
      buffer:
        imu_capacity: 2000
        measurement_capacity: 200
      process_noise:
        gyro: 0.015
        accel: 0.15
        gyro_bias: 1.54e-5
        accel_bias: 3.38e-4

    sync:
      gps: 0.005
      odom: 0.005
      wheel_odom: 0.005
      mag: 0.005
      baro: 0.005
      yaw: 0.005
      OOSM_tolerance: 0.005

    sensors:
      imu:
        enabled: true
        topic: /input/imu
        bias:
          gyro: {x: 0.0, y: 0.0, z: 0.0}
          accel: {x: 0.0, y: 0.0, z: 0.0}
        init:
          orientation:
            active: true
            gravity_tolerance: 0.5
            gyro_stationary_threshold: 0.1
            min_samples: 200
          bias:
            active: true

      gps:
        enabled: true
        topic: /input/gps/fix
        trust_covariance: true
        lever_arm: {x: 0.0, y: 0.0, z: 0.0}
        init:
          orientation:
            distance_threshold: 2.0
            delta_distance_threshold: 0.1
            max_speed: 30.0
            min_samples: 3
        continuous_orientation:
          active: false
          distance_threshold: 2.0
          delta_distance_threshold: 0.1
          max_speed: 30.0
          min_samples: 3

      odometry:
        enabled: false
        topic: /vio/odom
        trust_covariance:
          pose: true
          velocity: true

      wheel_odom:
        enabled: false
        topic: /input/wheel_odom
        type: twist_stamped

      baro:
        enabled: false
        topic: /input/baro

      mag:
        enabled: false
        topic: /input/mag
```

## Node Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/input/imu` | `sensor_msgs/Imu` | IMU measurements (required) |
| `/input/gps/fix` | `sensor_msgs/NavSatFix` | GPS fix (optional) |
| `/input/wheel_odom` | `nav_msgs/Odometry` or `geometry_msgs/TwistStamped` | Wheel odometry (optional) |
| `/vio/odom` | `nav_msgs/Odometry` | LIO/VIO odometry (optional) |
| `/input/mag` | `sensor_msgs/MagneticField` | Magnetometer (optional) |
| `/input/baro` | `sensor_msgs/FluidPressure` | Barometer (optional) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `~/odom` | `nav_msgs/Odometry` | Estimated odometry (state) |
| `~/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Estimated pose with covariance |
| `~/debug/gps_position` | `visualization_msgs/Marker` | GPS position visualization |
| `~/debug/lio_odom` | `nav_msgs/Odometry` | LIO/VIO odometry debug |
| `~/debug/yaw` | `visualization_msgs/Marker` | Yaw visualization |
| `tf` | `tf2_msgs/TFMessage` | Transform frames (world → body) |

### Frames

| Frame | Description |
|-------|-------------|
| `odom` | World/global frame (configurable) |
| `base_link` | Robot body frame (configurable) |

## Core Components

### INSEstimator
ROS 2 lifecycle node managing the entire estimation pipeline:
- Handles sensor subscriptions and message conversion
- Manages filter initialization and state propagation
- Implements OOSM handling with fixed-lag smoothing
- Publishes odometry, pose, and TF transforms

### iESEKF Filter
- Defines state representation on **SGal(3)** for pose + velocity + time
- Propagates state using IMU kinematics
- Updates state using GPS, odometry, wheel, magnetometer, and barometer measurements
- Filter definition using `lie_odyssey` API

### State
Represents system state: position, velocity, orientation, gravity, and IMU biases

### ENUConverter
Converts GPS LLA (latitude, longitude, altitude) coordinates to local ENU frame

### MeasurementHandler
Centralized measurement buffer with state history for OOSM replay

### Orientation Initializers
- **`IMUOrientationInitializer`**: Estimates initial roll/pitch from stationary IMU measurements
- **`GPSOrientationInitializer`**: Estimates initial yaw from GPS displacement

## Algorithm Overview

1. **Initialization**: Configure filter parameters and initialize orientation/bias
2. **IMU Propagation**: Queue IMU measurements and propagate state at fixed frequency
3. **ENU Setup**: First GPS fix initializes the local ENU datum
4. **Measurement Updates**: Process GPS, odometry, wheel, magnetometer, and barometer measurements as they arrive
5. **OOSM Handling**: Delayed measurements are replayed within the history window
6. **Publication**: Publish estimated pose, odometry, and TF transforms

## Tips and Troubleshooting

### IMU Calibration
- Keep the robot **stationary** during the initial orientation/bias estimation phase
- Disable specific calibration components if your IMU is pre-calibrated

### GPS Setup
- Ensure GPS has a valid fix before filter initialization
- The first GPS fix establishes the ENU datum — choose a location with good satellite visibility
- Adjust `lever_arm` parameters to match the physical GPS antenna offset

### Time Synchronization
- If sensors are not time synchronized, set `timing.use_receive_stamp: true`
- This stamps all measurements at receive time instead of trusting message headers

### Performance Tuning
- Adjust `filter.rate` to match your desired estimation frequency
- Increase `filter.buffer.imu_capacity` for longer offline processing
- Tune `filter.process_noise` parameters to match your sensor characteristics

### Visualization
Launch with RViz to visualize:
- Current estimated trajectory
- GPS positions
- Yaw estimates
- Estimated state and covariance

```bash
ros2 launch ins_ros ins_ros.launch.py rviz:=True
```

## License

See LICENSE file in the parent LieOdyssey repository.

## Maintainer

- **Author**: fetty
- **Email**: fetty3113@gmail.com

---

> ⚠️ **Note**: This file was automatically generated by an LLM model (`ling-3.0-flash-fin-free`). Content may not reflect the latest state of the repository — please verify against source code.
