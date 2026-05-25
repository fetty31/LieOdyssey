#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::wheel {

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, const iESEKF::Group& X_now, iESEKF::Measurement& r, iESEKF::HMat& H)
{
    /* To-Do:
        - get wheel odometry measurement in filter frame (e.g., ENU)
    */
    static State::V3 z_m = State::V3(0.0, 0.0, 0.0); // example wheel odometry measurement

    const int DoF = iESEKF::Group::Impl::DoF;

    r = iESEKF::Measurement::Zero(3);

    auto R = X_now.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate
    auto v = X_now.impl().subgroup<0>().linearVelocity();                  // velocity estimate

    auto z_hat = R.transpose() * v; // expected measurement in body frame

    r = z_m - z_hat;

    H = iESEKF::HMat::Zero(3, DoF);

    // velocity part
    H.block<3,3>(0, 3) = R.transpose();

    // orientation coupling: δθ × v
    H.block<3,3>(0, 6) = manif::skew(z_hat);
}

} // namespace ins_ros::iESEKF::wheel