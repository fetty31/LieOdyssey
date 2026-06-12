#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::pose {

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, 
            const iESEKF::Group& X_now, 
            const iESEKF::Group& Y, 
            iESEKF::Measurement& r, 
            iESEKF::HMat& H)
{
    /* To-Do:
        - get pose measurement (VIO, LIO, etc.) in filter frame (e.g., ENU)
    */
    using SGal3 = manif::SGal3<iESEKF::Scalar>;
    using Tangent = SGal3::Tangent;
    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;

    // Build dummy measurement SGal3 element
    // State::Quat q_meas = State::Quat::Identity(); // example orientation measurement
    // State::V3 p_meas = State::V3::Zero(); // example position measurement
    // State::V3 v_dummy = State::V3::Zero();

    // SGal3 Y(p_meas,                    // x y z                  0
    //         q_meas,                    // rotation               6
    //         v_dummy,                   // vx, vy, vz             3
    //         0.);  

    SGal3 Y_m = Y.impl().subgroup<0>(); // measurement as SGal3 (pose + velocity, but we will only use pose)

    // Group residual
    Tangent xi = Y_m.minus(X_now.impl().subgroup<0>()); // log(Y ⊞ X^{-1}) = log(Y * X^{-1})

    // Keep only rotation + position
    r = iESEKF::Measurement::Zero(6); // 10 for SGal3, but we will only use 6 (rotation + position)
    r.segment<3>(0) = xi.coeffs().segment<3>(0); // position
    r.segment<3>(3) = xi.coeffs().segment<3>(6); // rotation

    // Jacobian
    H = iESEKF::HMat::Zero(6, iESEKF::Group::Impl::DoF);

    H.block<3,3>(0,0) = Mat3::Identity();
    H.block<3,3>(3,6) = Mat3::Identity();
}

} // namespace ins_ros::iESEKF::pose