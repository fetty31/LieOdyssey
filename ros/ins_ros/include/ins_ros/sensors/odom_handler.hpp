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

    auto t_x = X_m.t();
    auto t_y = Y_m.t();

    // Y_mm = SGal3(Y_m.translation(),                    // x y z                  0
    //             Y_m.quat,                     // rotation               6
    //             state.v,                     // vx, vy, vz             3
    //             state.time),                 // delta t                9

    std::cout << std::setprecision(15) << "time x: " << t_x << std::endl;
    std::cout << std::setprecision(15) << "time y: " << t_y << std::endl;

    Tangent xi;
    SGal3::Jacobian J_X;
    SGal3::Jacobian J_Y;

    // xi = X_m.minus(Y_m, J_X, J_Y);

    // Residual: z - h(x) -> Y ⊖ X
    xi = Y_m.minus(X_m, J_Y, J_X);

    std::cout << std::setprecision(4) << "J_X:\n" << J_X << std::endl;
    std::cout << std::setprecision(4) << "J_Y:\n" << J_Y << std::endl;
    std::cout << std::setprecision(4) << "xi:\n" << xi.coeffs().transpose() << std::endl;

    r = xi.coeffs();

    H = iESEKF::HMat::Zero(
        10, 
        iESEKF::Group::Impl::DoF
    );

    // Derivative wrt filter state
    H.block<10,10>(0,0) = J_X;




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