#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::yaw {

/**
 * GPS heading measurement represented as a 2D unit vector:
 *
 *      y = [cos(yaw_gps)]
 *          [sin(yaw_gps)]
 *
 * This avoids the discontinuity of a scalar wrapped yaw residual
 * at +/- pi.
 */
// void H_fun(
//     const iESEKF::Filter& /*kf*/,
//     const iESEKF::Group& X_now,
//     const iESEKF::Scalar& y,
//     iESEKF::Measurement& r,
//     iESEKF::HMat& H)
// {
//     const int DoF = iESEKF::Group::Impl::DoF;

//     const auto R =
//         X_now.impl().subgroup<0>().quat().toRotationMatrix();

//     const auto euler = R.eulerAngles(2, 1, 0);

//     const iESEKF::Scalar yaw = euler(0);

//     std::cout << "YAW NOW: " << yaw*180.0/M_PI << std::endl;
//     std::cout << "YAW MEASUREMENT: " << y*180.0/M_PI  << std::endl;

//     // Predicted heading
//     //
//     // h(X) = [cos(yaw)]
//     //        [sin(yaw)]
//     iESEKF::Measurement h =
//         iESEKF::Measurement::Zero(2);
//     h(0) = std::cos(yaw);
//     h(1) = std::sin(yaw);

//     // Measured heading
//     iESEKF::Measurement z =
//         iESEKF::Measurement::Zero(2);
//     z(0) = std::cos(y);
//     z(1) = std::sin(y);

//     std::cout << "2D HEADING PREDICTED: " << h << std::endl;
//     std::cout << "2D HEADING MEASURED: " << z << std::endl;

//     // Residual: measurement - prediction
//     r = z - h;

//     // Measurement Jacobian
//     //
//     // dh/dyaw =
//     //     [-sin(yaw)]
//     //     [ cos(yaw)]
//     //
//     // With current right-perturbation convention and the
//     // simplified yaw Jacobian:
//     //
//     //     dyaw/d(delta_theta) = [0 0 1]
//     //
//     // therefore:
//     //
//     //     H(:,6:8) =
//     //
//     //     [0 0 -sin(yaw)]
//     //     [0 0  cos(yaw)]
//     //
//     H = iESEKF::HMat::Zero(2, DoF);

//     H(0, 6) = 0.0;
//     H(0, 7) = 0.0;
//     H(0, 8) = -std::sin(yaw);

//     H(1, 6) = 0.0;
//     H(1, 7) = 0.0;
//     H(1, 8) =  std::cos(yaw);


//     std::cout << "2D HEADING RESIDUAL: " << r << std::endl;
//     std::cout << "2D HEADING JACOBIAN: " << H << std::endl;

//     // with no simplified jacobian:
//     // const iESEKF::Scalar pitch = euler(1);
//     // const iESEKF::Scalar roll = euler(2);
//     // const iESEKF::Scalar cp = std::cos(pitch);

//     // H(0, 6) = 0.0;
//     // H(0, 7) = -std::sin(yaw) * std::sin(roll) / cp;
//     // H(0, 8) = -std::sin(yaw) * std::cos(roll) / cp;

//     // H(1, 6) = 0.0;
//     // H(1, 7) =  std::cos(yaw) * std::sin(roll) / cp;
//     // H(1, 8) =  std::cos(yaw) * std::cos(roll) / cp;
// }

void H_fun(
    const iESEKF::Filter& /*kf*/,
    const iESEKF::Group& X_now,
    const iESEKF::Scalar& y,
    iESEKF::Measurement& r,
    iESEKF::HMat& H)
{
    constexpr int DoF = iESEKF::Group::Impl::DoF;

    const auto R =
        X_now.impl().subgroup<0>().quat().toRotationMatrix();

    // Predicted horizontal heading
    const auto forward = R.col(0);

    iESEKF::Measurement h =
        iESEKF::Measurement::Zero(2);

    h(0) = forward.x();
    h(1) = forward.y();
    h.normalize();

    // GPS heading
    iESEKF::Measurement z =
        iESEKF::Measurement::Zero(2);

    z(0) = std::cos(y);
    z(1) = std::sin(y);

    // Residual
    r = z - h;

    std::cout << "2D HEADING PREDICTED: " << h << std::endl;
    std::cout << "2D HEADING MEASURED: " << z << std::endl;
    std::cout << "2D HEADING RESIDUAL: " << r << std::endl;

    // Jacobian
    H = iESEKF::HMat::Zero(2, DoF);

    const auto J_forward = -R * manif::skew(
        State::V3(1.0, 0.0, 0.0));

    H.block<2, 3>(0, 6) =
        J_forward.block<2, 3>(0, 0);
}

} // namespace ins_ros::iESEKF::yaw