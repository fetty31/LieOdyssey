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
    using SGal3 = manif::SGal3<iESEKF::Scalar>;
    using Tangent = SGal3::Tangent;

    SGal3 X_m = X_now.impl().subgroup<0>();
    SGal3 Y_m = Y.impl().subgroup<0>();

    Tangent xi;
    SGal3::Jacobian J_X;
    SGal3::Jacobian J_Y;

    xi = X_m.minus(Y_m, J_X, J_Y);

    std::cout << "J_X:\n" << J_X << std::endl;
    std::cout << "J_Y:\n" << J_Y << std::endl;
    std::cout << "xi:\n" << xi.coeffs().transpose() << std::endl;

    r = iESEKF::Measurement::Zero(9);
    r.segment<3>(0) = xi.coeffs().segment<3>(0);
    r.segment<3>(3) = xi.coeffs().segment<3>(3);
    r.segment<3>(6) = xi.coeffs().segment<3>(6);

    H = iESEKF::HMat::Zero(
        9,
        iESEKF::Group::Impl::DoF
    );

    // Derivative wrt filter state
    H.block<9, 9>(0, 0) = J_X.block<9, 9>(0, 0);




    // using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;

    // SGal3 Y_m = Y.impl().subgroup<0>(); // measurement as SGal3 (pose + velocity)

    // // Group residual (right-perturbation)
    // Tangent xi = X_now.impl().subgroup<0>().minus(Y_m); // log(X^{-1} * Y)

    // r = iESEKF::Measurement::Zero(9); // following manif::SGal3 order: 3 position + 3 velocity + 3 rotation (+1 delta time, but we will ignore it for now)
    // r.segment<3>(0) = xi.coeffs().segment<3>(0); // position
    // r.segment<3>(3) = xi.coeffs().segment<3>(3); // velocity
    // r.segment<3>(6) = xi.coeffs().segment<3>(6); // rotation

    // // Jacobian
    // H = iESEKF::HMat::Zero(9, iESEKF::Group::Impl::DoF);
    // H.block<3,3>(0,0) = Mat3::Identity(); // dp/dp = Identity (position)
    // H.block<3,3>(3,3) = Mat3::Identity(); // dv/dv = Identity (velocity)
    // H.block<3,3>(6,6) = Mat3::Identity(); // dq/dq = Identity (rotation)

}

} // namespace ins_ros::iESEKF::odom