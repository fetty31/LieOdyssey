#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::yaw {

inline iESEKF::Scalar wrap_to_pi(iESEKF::Scalar angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

void H_fun(
    const iESEKF::Filter& /*kf*/,
    const iESEKF::Group& X_now,
    const iESEKF::Scalar& y,
    iESEKF::Measurement& r,
    iESEKF::HMat& H)
{
    const int DoF = iESEKF::Group::Impl::DoF;

    // Predicted yaw
    auto R = X_now.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate

    const auto euler = R.matrix().eulerAngles(2, 1, 0);
    const iESEKF::Scalar yaw   = euler(0);
    const iESEKF::Scalar pitch = euler(1);

    // Residual
    r = iESEKF::Measurement::Zero(1);

    r(0) = wrap_to_pi(y - yaw);

    // Jacobian
    H = iESEKF::HMat::Zero(1, DoF);
    H(0, 6) = std::cos(yaw) * std::tan(pitch);
    H(0, 7) = std::sin(yaw) * std::tan(pitch);
    H(0, 8) = 1.0;
}

} // namespace ins_ros::iESEKF::yaw