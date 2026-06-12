#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::barometer {

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, 
            const iESEKF::Group& X_now, 
            const iESEKF::Scalar& y, 
            iESEKF::Measurement& r, 
            iESEKF::HMat& H)
{
    const int DoF = iESEKF::Group::Impl::DoF;

    r = iESEKF::Measurement::Zero(1);

    // Extract position
    auto p = X_now.impl().subgroup<0>().translation();

    r(0) = y - p.z(); // residual

    H = iESEKF::HMat::Zero(1, DoF);

    // derivative wrt position z
    H(0, 2) = 1.0;
}

} // namespace ins_ros::iESEKF::barometer