#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::barometer {

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, const iESEKF::Group& X_now, iESEKF::Measurement& r, iESEKF::HMat& H)
{
    /* To-Do:
        - get barometric measurement (z in filter frame, e.g., ENU)
    */
    double z_m = 0.0; 

    const int DoF = iESEKF::Group::Impl::DoF;

    r = iESEKF::Measurement::Zero(1);

    // Extract position
    State::V3 p = X_now.impl().subgroup<0>().translation();

    r(0) = z_m - p.z(); // residual

    H = iESEKF::HMat::Zero(1, DoF);

    // derivative wrt position z
    H(0, 2) = 1.0;
}

} // namespace ins_ros::iESEKF::barometer