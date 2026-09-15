#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::relative_odom {

struct RelativeOdomMeasurement
{
    // Fixed filter state at reference time t_ref.
    iESEKF::Group X_ref;

    // Odometry pose at reference time t_ref.
    iESEKF::Group Y_ref;

    // Odometry pose at current time t_cur.
    iESEKF::Group Y_cur;

    double t_ref;
    double t_cur;
};

/**
 * Relative odometry measurement with a fixed reference state.
 *
 * States:
 *   Xi : filter state at reference time t_i (kept fixed)
 *   Xj : current filter state at time t_j (updated)
 *
 * Measured relative motion:
 *
 *      Z_ij = Y_i^{-1} * Y_j
 *
 * Predicted relative motion:
 *
 *      X_ij = Xi^{-1} * Xj
 *
 * Residual (right-minus convention):
 *
 *      r = Z_ij ⊖ X_ij
 *
 * Since the filter uses right perturbations:
 *
 *      Xj⁺ = Xj * Exp(δx)
 *
 * and Xi is treated as constant:
 *
 *      X_ij⁺
 *        = Xi^{-1} * Xj * Exp(δx)
 *        = X_ij * Exp(δx)
 *
 * Therefore, the perturbation of Xj propagates directly
 * to the relative state X_ij.
 *
 * If manif provides:
 *
 *      J_X = ∂(Z_ij ⊖ X_ij) / ∂δX_ij
 *
 * then:
 *
 *      H = ∂r / ∂δx
 *        = J_X
 *
 * This formulation avoids state augmentation because Xi is
 * considered a fixed historical state and only Xj is updated.
 */

void H_fun(
    const iESEKF::Filter& /*kf*/,
    const iESEKF::Group& X_now,
    const RelativeOdomMeasurement& Y,
    iESEKF::Measurement& r,
    iESEKF::HMat& H)
{
    using SGal3 = manif::SGal3<iESEKF::Scalar>;
    using Tangent = SGal3::Tangent;

    const auto X_i = Y.X_ref;
    const auto X_j = X_now;
    const auto Y_i = Y.Y_ref;
    const auto Y_j = Y.Y_cur;

    // Filter states
    SGal3 Xi = X_i.impl().subgroup<0>();
    SGal3 Xj = X_j.impl().subgroup<0>();

    // Odometry measurements
    SGal3 Yi = Y_i.impl().subgroup<0>();
    SGal3 Yj = Y_j.impl().subgroup<0>();

    SGal3 Yij = Yi.inverse() * Yj;
    SGal3 Xij = Xi.inverse() * Xj;

    Tangent xi;
    SGal3::Jacobian J_Yij;
    SGal3::Jacobian J_Xij;

    xi = Yij.minus(Xij, J_Yij, J_Xij);

    r = xi.coeffs();

    H = iESEKF::HMat::Zero(
        10,
        iESEKF::Group::Impl::DoF
    );

    H.block<10, 10>(0, 0) = J_Xij;

}

} // namespace ins_ros::iESEKF::relative_odom