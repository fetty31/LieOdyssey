
#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::yaw {

/**
 * GPS heading measurement represented as a 2D unit vector:
 *
 *      z = [cos(yaw_gps), sin(yaw_gps)]
 *
 * The predicted heading is obtained by projecting the
 * body forward axis onto the horizontal plane and normalizing.
 */
void H_fun(
    const iESEKF::Group& X_now,
    const iESEKF::Scalar& y,
    iESEKF::Measurement& r,
    iESEKF::HMat& H)
{
    constexpr int DoF = iESEKF::Group::Impl::DoF;

    const auto R =
        X_now.impl().subgroup<0>().quat().toRotationMatrix();

    // Predicted forward direction in world frame.
    const State::V3 ex(1.0, 0.0, 0.0);
    const State::V3 forward = R * ex;

    // Horizontal projection.
    Eigen::Matrix<iESEKF::Scalar, 2, 3> P;
    P << 1, 0, 0,
         0, 1, 0;

    const Eigen::Matrix<iESEKF::Scalar, 2, 1> p =
        P * forward;

    const auto norm = p.norm();

    // Avoid undefined heading when forward axis is
    // approximately vertical.
    if (norm < 1e-6) {
        r.setZero();
        H.setZero();
        return;
    }

    // Normalized predicted heading.
    const Eigen::Matrix<iESEKF::Scalar, 2, 1> h =
        p / norm;

    // GPS heading measurement.
    Eigen::Matrix<iESEKF::Scalar, 2, 1> z;
    z << std::cos(y), std::sin(y);

    // Residual.
    r = z - h;

    // Derivative of forward direction.
    const Eigen::Matrix<iESEKF::Scalar, 3, 3> J_forward =
        -R * manif::skew(ex);

    // Derivative of normalized horizontal projection.
    const Eigen::Matrix<iESEKF::Scalar, 2, 2> J_norm =
        (Eigen::Matrix<iESEKF::Scalar, 2, 2>::Identity()
         - h * h.transpose()) / norm;

    const Eigen::Matrix<iESEKF::Scalar, 2, 3> J_heading =
        J_norm * P * J_forward;

    // Measurement Jacobian.
    H = iESEKF::HMat::Zero(2, DoF);

    H.block<2, 3>(0, 6) = J_heading;
}

} // namespace ins_ros::iESEKF::yaw
