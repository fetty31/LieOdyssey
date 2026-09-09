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
    // using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;

    SGal3 X_m = X_now.impl().subgroup<0>();
    SGal3 Y_m = Y.impl().subgroup<0>();

    Tangent xi;
    SGal3::Jacobian J_X;
    SGal3::Jacobian J_Y;

    xi = X_m.minus(Y_m, J_X, J_Y); 

    r = iESEKF::Measurement::Zero(6); // measurement as SGal3 (pose + velocity, but we will only use pose)
    r.segment<3>(0) = xi.coeffs().segment<3>(0); // position
    r.segment<3>(3) = xi.coeffs().segment<3>(6); // rotation

    H = iESEKF::HMat::Zero(
        6,
        iESEKF::Group::Impl::DoF
    );

    // Derivative wrt filter state
    H.block<3, 3>(0, 0) = J_X.block<3, 3>(0, 0);
    H.block<3, 3>(3, 6) = J_X.block<3, 3>(6, 6);

    
    // SGal3 Y_m = Y.impl().subgroup<0>(); // measurement as SGal3 (pose + velocity, but we will only use pose)

    // // Group residual
    // Tangent xi = Y_m.minus(X_now.impl().subgroup<0>()); // log(Y ⊞ X^{-1}) = log(Y * X^{-1})

    // // Keep only rotation + position
    // r = iESEKF::Measurement::Zero(6); // 10 for SGal3, but we will only use 6 (rotation + position)
    // r.segment<3>(0) = xi.coeffs().segment<3>(0); // position
    // r.segment<3>(3) = xi.coeffs().segment<3>(6); // rotation

    // // Jacobian
    // H = iESEKF::HMat::Zero(6, iESEKF::Group::Impl::DoF);

    // H.block<3,3>(0,0) = Mat3::Identity();
    // H.block<3,3>(3,6) = Mat3::Identity();
}

} // namespace ins_ros::iESEKF::pose