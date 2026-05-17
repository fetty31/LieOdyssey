#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::magnetometer {

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, const iESEKF::Group& X_now, iESEKF::Measurement& r, iESEKF::HMat& H);

} // namespace ins_ros::iESEKF::magnetometer