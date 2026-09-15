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
    // h.normalize();

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

    State::V3 ex(1.0, 0.0, 0.0);
    Eigen::Matrix<iESEKF::Scalar, 3, 3> J_forward = -R * manif::skew(ex);

    std::cout << "J_forward: " << J_forward << std::endl;
    std::cout << "R: " << R << std::endl;
    std::cout << "skew(ex): " << manif::skew(ex) << std::endl;

    H.block<2, 3>(0, 6) =
        J_forward.block<2, 3>(0, 0);

    std::cout << "H jacob: " << H << std::endl;
}

// void H_fun(
//     const iESEKF::Filter& /*kf*/,
//     const iESEKF::Group& X_now,
//     const iESEKF::Scalar& y,
//     iESEKF::Measurement& r,
//     iESEKF::HMat& H)
// {
//     constexpr int DoF = iESEKF::Group::Impl::DoF;

//     const auto R =
//         X_now.impl().subgroup<0>().quat().toRotationMatrix();

//     // Body forward axis
//     const State::V3 ex(1.0, 0.0, 0.0);

//     // World forward direction
//     const State::V3 forward = R.col(0);

//     // Horizontal projection
//     iESEKF::Measurement u;
//     u << forward.x(), forward.y();

//     const iESEKF::Scalar norm = u.norm();

//     // Normalized horizontal heading
//     iESEKF::Measurement h = u / norm;

//     // GPS heading
//     iESEKF::Measurement z;
//     z << std::cos(y), std::sin(y);

//     // Residual
//     r = z - h;

//     // Jacobian of R * ex
//     Eigen::Matrix<iESEKF::Scalar, 3, 3> J_forward =
//         -R * manif::skew(ex);

//     // Jacobian of horizontal projection
//     Eigen::Matrix<iESEKF::Scalar, 2, 3> J_u =
//         J_forward.topRows<2>();

//     // Jacobian of normalization u / ||u||
//     Eigen::Matrix<iESEKF::Scalar, 2, 2> J_norm =
//         (Eigen::Matrix<iESEKF::Scalar, 2, 2>::Identity() 
//         - h * h.transpose()) / norm;

//     H = iESEKF::HMat::Zero(2, DoF);

//     H.block<2, 3>(0, 6) =
//         J_norm * J_u;
// }

} // namespace ins_ros::iESEKF::yaw