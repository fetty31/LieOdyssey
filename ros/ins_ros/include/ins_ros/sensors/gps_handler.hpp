#pragma once

#include "ins_ros/ekf.hpp"
#include <cmath>

namespace ins_ros::iESEKF::gps {

struct GPSMeasurement
{
    State::V3 position_enu = State::V3::Zero(); // GPS position in ENU frame
    State::V3 lever_arm = State::V3::Zero(); // GPS position relative to body, expressed in body frame
};

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, 
            const iESEKF::Group& X_now, 
            const GPSMeasurement& y, 
            iESEKF::Measurement& r, 
            iESEKF::HMat& H)
{
    using Scalar = iESEKF::Scalar;
    
    const int DoF = iESEKF::Group::Impl::DoF;

    // Measurement: position only (3D GPS fix in ENU)
    r = iESEKF::Measurement::Zero(3);

    // Extract position & orientation estimate in world frame (ENU)
    State::V3 p_hat = X_now.impl().subgroup<0>().translation();
    auto R_hat = X_now.impl().subgroup<0>().quat().toRotationMatrix();

    State::V3 p_gps_hat = p_hat + R_hat * y.lever_arm;
    
    // Assume measurement is already converted to ENU frame
    r.segment<3>(0) = y.position_enu - p_gps_hat;

    // Jacobian
    H = iESEKF::HMat::Zero(3, DoF);

    // dp/dp = Identity
    H.block<3,3>(0, 0) = Eigen::Matrix<Scalar, 3, 3>::Identity(); 

    // dp/dq = -R_hat * skew(lever_arm)
    H.block<3,3>(0, 6) = -R_hat * manif::skew(y.lever_arm);
}

} // namespace ins_ros::iESEKF::gps