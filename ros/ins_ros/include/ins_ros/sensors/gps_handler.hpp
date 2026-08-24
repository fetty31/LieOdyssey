#pragma once

#include "ins_ros/ekf.hpp"
#include <cmath>

namespace ins_ros::iESEKF::gps {

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, 
            const iESEKF::Group& X_now, 
            const iESEKF::Measurement& y, 
            iESEKF::Measurement& r, 
            iESEKF::HMat& H)
{
    using Scalar = iESEKF::Scalar;
    
    const int DoF = iESEKF::Group::Impl::DoF;

    // Measurement: position only (3D GPS fix in ENU)
    r = iESEKF::Measurement::Zero(3);

    // Extract position estimate in world frame
    State::V3 p_hat = X_now.impl().subgroup<0>().translation();
    
    // Assume measurement is already converted to ENU frame
    State::V3 y_enu = y.head<3>().cast<Scalar>();

    r.segment<3>(0) = y_enu - p_hat;

    // Jacobian
    H = iESEKF::HMat::Zero(3, DoF);

    // dp/dp = Identity
    H.block<3,3>(0, 0) = Eigen::Matrix<Scalar, 3, 3>::Identity(); 
}

} // namespace ins_ros::iESEKF::gps