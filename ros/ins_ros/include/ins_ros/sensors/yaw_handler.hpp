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

    // -------------------------------------------------------------------------
    // Measurement model
    //
    // Predicted heading direction in the world frame:
    //
    //      forward = R e_x
    //
    // Only the horizontal components are measured.
    // -------------------------------------------------------------------------
    const auto forward = R.col(0);

    iESEKF::Measurement h =
        iESEKF::Measurement::Zero(2);

    h(0) = forward.x();
    h(1) = forward.y();
    // h.normalize();

    // Measured heading direction.
    iESEKF::Measurement z =
        iESEKF::Measurement::Zero(2);
    z(0) = std::cos(y);
    z(1) = std::sin(y);

    // Residual:
    //      r = z - h(X)
    r = z - h;

    // -------------------------------------------------------------------------
    // Jacobian
    //
    // Right perturbation:
    //
    //      R+ = R Exp(dtheta)
    //
    // Therefore:
    //
    //      d(Re_x)/dtheta = -R [e_x]x
    //
    //      (H = dh/dtheta)
    //
    // -------------------------------------------------------------------------

    // Jacobian
    H = iESEKF::HMat::Zero(2, DoF);

    State::V3 ex(1.0, 0.0, 0.0);
    Eigen::Matrix<iESEKF::Scalar, 3, 3> J_forward = -R * manif::skew(ex);

    H.block<2, 3>(0, 6) =
        J_forward.block<2, 3>(0, 0);
}

} // namespace ins_ros::iESEKF::yaw