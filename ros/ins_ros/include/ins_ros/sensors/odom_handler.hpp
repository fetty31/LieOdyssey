#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::odom {

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, 
            const iESEKF::Group& X_now, 
            const iESEKF::Group& Y, 
            iESEKF::Measurement& r, 
            iESEKF::HMat& H)
{
    /* To-Do:
        - get full odometry measurement (VIO, LIO, etc.) in filter frame (e.g., ENU)
    */
    using SGal3 = manif::SGal3<iESEKF::Scalar>;
    using Tangent = SGal3::Tangent;
    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;

    using SGal3 = manif::SGal3<iESEKF::Scalar>;
    using Tangent = SGal3::Tangent;
    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;
    
    SGal3 Y_m = Y.impl().subgroup<0>(); // measurement as SGal3 (pose + velocity)

    // Group residual
    Tangent xi = Y_m.minus(X_now.impl().subgroup<0>()); // log(Y ⊞ X^{-1}) = log(Y * X^{-1})

    r = iESEKF::Measurement::Zero(10); // following manif::SGal3 order: 3 position + 3 velocity + 3 rotation + 1 delta t
    r.segment<3>(0) = xi.coeffs().segment<3>(0); // position
    r.segment<3>(3) = xi.coeffs().segment<3>(3); // velocity
    r.segment<3>(6) = xi.coeffs().segment<3>(6); // rotation

    // Jacobian
    H = iESEKF::HMat::Zero(10, iESEKF::Group::Impl::DoF);
    H.block<3,3>(0,0) = Mat3::Identity(); // dp/dp = Identity (position)
    H.block<3,3>(3,3) = Mat3::Identity(); // dv/dv = Identity (velocity)
    H.block<3,3>(6,6) = Mat3::Identity(); // dq/dq = Identity (rotation)

}

} // namespace ins_ros::iESEKF::odom