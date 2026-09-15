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

    // Residual: z - h(x) -> Y ⊖ X
    xi = Y_m.minus(X_m, J_Y, J_X);

    r = xi.coeffs();

    H = iESEKF::HMat::Zero(
        10, 
        iESEKF::Group::Impl::DoF
    );

    // Derivative wrt filter state
    H.block<10,10>(0,0) = -J_X;
}

} // namespace ins_ros::iESEKF::odom