#pragma once

// Project includes
#include "ins_ros/ekf.hpp"
#include "ins_ros/sensors/gps_handler.hpp"

#include <Eigen/Dense>
#include <optional>

namespace ins_ros::measurements {

// --- Stamped measurement types (all times in ROS header seconds) ---

struct StampedGps {
  double stamp{-1.0};
  iESEKF::gps::GPSMeasurement meas{};
  Eigen::Matrix<iESEKF::Scalar, 3, 3> R{Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity()};
  Eigen::Matrix<iESEKF::Scalar, 3, 3> R_inv{Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity()};
};

struct StampedOdom {
  double stamp{-1.0};
  iESEKF::Group group{};
  Eigen::Matrix<iESEKF::Scalar, 10, 10> R{Eigen::Matrix<iESEKF::Scalar, 10, 10>::Identity()};
  Eigen::Matrix<iESEKF::Scalar, 10, 10> R_inv{Eigen::Matrix<iESEKF::Scalar, 10, 10>::Identity()};
  bool has_velocity{true};
};

struct StampedWheel {
  double stamp{-1.0};
  iESEKF::Measurement meas{iESEKF::Measurement::Zero(3)};
  Eigen::Matrix<iESEKF::Scalar, 3, 3> R{Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity()};
  Eigen::Matrix<iESEKF::Scalar, 3, 3> R_inv{Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity()};
};

struct StampedMag {
  double stamp{-1.0};
  iESEKF::Measurement meas{iESEKF::Measurement::Zero(3)};
  Eigen::Matrix<iESEKF::Scalar, 3, 3> R{Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity() * 0.05};
  Eigen::Matrix<iESEKF::Scalar, 3, 3> R_inv{Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity() * 20.0};
};

struct StampedBaro {
  double stamp{-1.0};
  iESEKF::Scalar pressure{101325.0};
  Eigen::Matrix<iESEKF::Scalar, 1, 1> R{Eigen::Matrix<iESEKF::Scalar, 1, 1>::Identity()};
  Eigen::Matrix<iESEKF::Scalar, 1, 1> R_inv{Eigen::Matrix<iESEKF::Scalar, 1, 1>::Identity()};
};

struct StampedYaw {
  double stamp{-1.0};
  iESEKF::Scalar yaw{0.0};
  Eigen::Matrix<iESEKF::Scalar, 2, 2> R{Eigen::Matrix<iESEKF::Scalar, 2, 2>::Identity() * 0.01};
  Eigen::Matrix<iESEKF::Scalar, 2, 2> R_inv{Eigen::Matrix<iESEKF::Scalar, 2, 2>::Identity() * 100.0};
};

/**
 * @brief State snapshot used for delayed-measurement rewind/repropagation.
 */
struct StateSnapshot {
  double stamp{-1.0};
  iESEKF::Group group{};
  iESEKF::MatDoF covariance{iESEKF::MatDoF::Identity() * 1e-3};
};

}  // namespace ins_ros::measurements
