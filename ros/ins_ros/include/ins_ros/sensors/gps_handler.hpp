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

    // Extract SGal3 estimate in world frame (ENU)
    iESEKF::Bundle s = X_now.impl(); 
	manif::SGal3<Scalar> SGal3_s = s.subgroup<0>();

	// Compute prediction
	Eigen::Matrix<Scalar, 3, manif::SGal3<Scalar>::DoF> J_act;
    State::V3 p_gps_hat = SGal3_s.act(
        y.lever_arm,
        J_act
    );

    // Residual in ENU frame
    r.segment<3>(0) = y.position_enu - p_gps_hat;

    // Jacobian
    H = iESEKF::HMat::Zero(3, DoF);
    H.block<3, manif::SGal3<Scalar>::DoF>(0,0) = J_act;
}

} // namespace ins_ros::iESEKF::gps