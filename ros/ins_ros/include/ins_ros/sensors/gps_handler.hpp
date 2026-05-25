#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::gps {

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, const iESEKF::Group& X_now, iESEKF::Measurement& r, iESEKF::HMat& H)
{
    /* To-Do:
        - get gps measurement in filter frame (e.g., ENU)
    */
    static Eigen::Vector3d z_m = Eigen::Vector3d(0.0, 0.0, 0.0); // example GPS measurement

    using Scalar = iESEKF::Scalar;
    
    const int DoF = iESEKF::Group::Impl::DoF;

    // Measurement: position only (3D GPS fix)
    r = iESEKF::Measurement::Zero(3);

    // Extract position
    State::V3 z_hat = X_now.impl().subgroup<0>().translation();

    r.segment<3>(0) = z_m - z_hat;

    // Jacobian
    H = iESEKF::HMat::Zero(3, DoF);

    // dp/dp = Identity
    H.block<3,3>(0, 0) = Eigen::Matrix<Scalar, 3, 3>::Identity(); 
}

} // namespace ins_ros::iESEKF::gps