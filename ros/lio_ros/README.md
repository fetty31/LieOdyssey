# LIO-ROS: LiDAR-Inertial Odometry

A minimalist LiDAR-Inertial Odometry (LIO) implementation for ROS 2, featuring EKF-based SLAM with octree-based 3D mapping. This package provides robust odometry estimation by fusing LiDAR scans with IMU measurements using the Iterative Error-State Extended Kalman Filter (iESEKF) from `LieOdyssey`.

## Features

- **LiDAR-Inertial Fusion**: Combines LiDAR point clouds with IMU data for robust odometry
- **EKF-based State Estimation**: Uses iESEKF on Lie groups for accurate pose tracking
- **Octree-based Mapping**: Efficient spatial data structure for 3D map representation
- **Point-to-Plane Registration**: Utilizes plane constraints from LiDAR data for registration
- **Multi-Sensor Support**: Supports multiple LiDAR types:
  - Ouster
  - Velodyne
  - Hesai
  - Livox
- **Motion Compensation**: Deskews LiDAR scans based on IMU motion during acquisition
- **Online IMU Calibration**: Automatic estimation of gravity, accelerometer bias, and gyroscope bias
- **ROS 2 Integration**: Full ROS 2 middleware support with frame broadcasting and visualization

## Prerequisites

### System Requirements
- Ubuntu 20.04 or later
- ROS 2 Humble or later
- C++17 compatible compiler

### Dependencies
- **lie_odyssey**: Lie group-based library for state-space estimation
- **PCL** (Point Cloud Library) >= 1.12
- **Eigen3**: Linear algebra library
- **OpenMP**: For multi-threaded processing

### ROS 2 Dependencies
- `rclcpp`: ROS C++ client library
- `sensor_msgs`: Sensor message types
- `geometry_msgs`: Geometry message types
- `visualization_msgs`: Visualization message types
- `nav_msgs`: Navigation message types
- `tf2` and `tf2_ros`: Transform library and ROS integration
- `pcl_conversions`: PCL and ROS message conversions

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
   colcon build --packages-select lio_ros
   ```

4. **Source the setup script**:
   ```bash
   source install/setup.bash
   ```

## Usage

### Launch the Node

Run the LIO-ROS odometry node:

```bash
ros2 launch lio_ros lio_ros.launch.py
```

#### Launch Arguments

- `config`: Path to YAML configuration file (default: `config/params.yaml`)
- `rviz`: Enable RViz visualization (default: `False`)

Example with RViz:
```bash
ros2 launch lio_ros lio_ros.launch.py rviz:=True
```

Example with custom config:
```bash
ros2 launch lio_ros lio_ros.launch.py config:=/path/to/custom_config.yaml
```

## Configuration

### Configuration File Structure

The package uses YAML configuration files located in the `config/` directory. Key configuration parameters:

```yaml
lio_ros_node:
  ros__parameters:
    # Topic subscriptions
    topics:
      input:
        lidar: /ouster/points        # LiDAR point cloud topic
        imu: /ouster/imu              # IMU measurements topic

    # Output frames and transforms
    frames:
      world: map                      # World/global frame name
      body: base_link                 # Robot body frame name
      tf_pub: true                    # Publish TF transforms

    # Processing parameters
    num_threads: 1                    # OpenMP threads for parallel processing
    sensor_type: "OUSTER"             # LiDAR sensor type
    debug: true                       # Enable debug point clouds
    time_offset: true                 # Estimate sync offset between IMU and LiDAR
    end_of_sweep: false               # Sweep reference time (start vs end)
    motion_compensation: true         # Enable LiDAR deskewing

    # Automatic IMU calibration
    calibration:
      gravity_align: true             # Estimate gravity vector
      accel: true                     # Estimate accelerometer bias
      gyro: true                      # Estimate gyroscope bias
      time: 3.0                       # [s] Calibration duration (robot must be stationary)
```

### Pre-configured Datasets

Two example configurations are provided:

- **`config/kitti.yaml`**: Calibrated for the [KITTI](https://www.cvlibs.net/datasets/kitti/) dataset
- **`config/ona.yaml`**: Calibrated for the [ONA](https://www.vaivelogistics.com/) robot
- **`config/params.yaml`**: General purpose configuration

## Node Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/ouster/points` | `sensor_msgs/PointCloud2` | LiDAR point cloud (configurable) |
| `/ouster/imu` | `sensor_msgs/Imu` | IMU measurements (configurable) |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `tf` | `tf2_msgs/TFMessage` | Transform frames (world → body) |
| `/lio_ros_node/odometry` | `nav_msgs/Odometry` | Estimated odometry |

### Frames

| Frame | Description |
|-------|-------------|
| `map` | Global/world frame (configurable) |
| `base_link` | Robot body frame (configurable) |

## Core Components

### OdometryCore
Main processing engine that:
- Initializes and manages the SLAM system
- Processes IMU measurements for state propagation
- Processes LiDAR scans for state updates
- Manages state and map representations

### EKF (Extended Kalman Filter)
- Defines state representation on Lie groups for `pose` + `velocity` + `IMU bias` + `gravity`
- Propagates state using IMU kinematics
- Updates state using point-to-plane measurements
- Filter definition with `LieOdyssey` API. 

### Map
- Maintains 3D map using octree structure
- Provides efficient spatial queries
- Supports registration (quick plane estimation)

### State
- Represents system state: pose, velocity, biases
- Manages extrinsic calibration between LiDAR and IMU
- Handles covariance matrices

## Algorithm Overview

1. **IMU Pre-processing**: Queue IMU measurements and synchronize with LiDAR
2. **Deskewing**: Compensate for robot motion during LiDAR acquisition using IMU
3. **Registration**: Match deskewed point cloud to existing map using point-to-plane residuals
4. **EKF Propagation**: Update state prediction using incoming IMU measurements
5. **EKF Update**: Correct state estimate using registration residuals as measurements
6. **Map Update**: Add registered scan to global octree map

## Tips and Troubleshooting

### IMU Calibration
- Keep the robot **stationary** for the duration specified in `calibration.time` (default: 3 seconds) during startup
- The calibration phase estimates gravity alignment and sensor biases
- Disable specific calibration components if your IMU is pre-calibrated

### Time Synchronization
- If LiDAR and IMU are not time synchronized, set `time_offset: true`
- Enable `motion_compensation: true` to deskew scans based on IMU rotation

### LiDAR Configuration
- Set `sensor_type` to match your LiDAR: `OUSTER`, `VELODYNE`, `HESAI`, or `LIVOX`
- Adjust `end_of_sweep` based on whether your LiDAR timestamps refer to the start or end of a scan

### Performance Tuning
- Increase `num_threads` for faster processing on multi-core systems
- Disable `debug: false` to reduce computational overhead after tuning

### Visualization
Launch with RViz to visualize:
- Current odometry trajectory
- Point clouds (deskewed, registered, map)
- Estimated state and covariance

```bash
ros2 launch lio_ros lio_ros.launch.py rviz:=True
```

## License

See LICENSE file in the parent LieOdyssey repository.

## Maintainer

- **Author**: fetty
- **Email**: fetty3113@gmail.com