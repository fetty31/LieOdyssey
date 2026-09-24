// #pragma once

// #include "ins_ros/ekf.hpp"

// namespace ins_ros::iESEKF::wheel {

// // Measurement function
// void H_fun(const iESEKF::Filter& /*kf*/, 
//             const iESEKF::Group& X_now, 
//             const iESEKF::Measurement& y, 
//             iESEKF::Measurement& r, 
//             iESEKF::HMat& H)
// {
//     const int DoF = iESEKF::Group::Impl::DoF;

//     r = iESEKF::Measurement::Zero(3);

//     auto R = X_now.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate
//     auto v = X_now.impl().subgroup<0>().linearVelocity();           // velocity estimate

//     auto z_hat = R.transpose() * v; // expected measurement in body frame

//     r = y - z_hat;

//     H = iESEKF::HMat::Zero(3, DoF);

//     // Jacobian H = dh = R^T dv - skew(z_hat) dθ

//     // velocity part (dh/dv)
//     H.block<3,3>(0, 3) = R.transpose();

//     // orientation part (dh/dθ)
//     H.block<3,3>(0, 6) = -manif::skew(z_hat);
// }

// } // namespace ins_ros::iESEKF::wheel

#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::wheel {

void H_fun(const iESEKF::Filter& /*kf*/,
           const iESEKF::Group& X_now,
           const iESEKF::Measurement& y,
           iESEKF::Measurement& r,
           iESEKF::HMat& H)
{
    using SGal3 = manif::SGal3<iESEKF::Scalar>;

    constexpr int DoF = iESEKF::Group::Impl::DoF;

    const SGal3 X_m =
        X_now.impl().template subgroup<0>();

    const auto R = X_m.rotation();
    const auto v = X_m.linearVelocity();

    const auto z_hat = R.transpose() * v;

    r = y - z_hat;

    H = iESEKF::HMat::Zero(3, DoF);

    // Manif SGal3 tangent ordering:
    //
    //   [rho(3), nu(3), theta(3), s(1)]
    //
    // h(X) = R^T v
    //
    // dh/d(delta) =
    //
    //   [ 0  I  skew(z_hat)  0 ]

    // velocity part (dh/dv)
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    // orientation part (dh/dθ)
    H.block<3, 3>(0, 6) = manif::skew(z_hat);
}

} // namespace ins_ros::iESEKF::wheel