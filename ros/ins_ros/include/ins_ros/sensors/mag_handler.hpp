#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::magnetometer {

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, const iESEKF::Group& X_now, iESEKF::Measurement& r, iESEKF::HMat& H)
{
    /* To-Do:
        - get magnetic field measurement 
    */
    State::V3 z_m = State::V3(0.0, 0.0, 0.0); // example magnetometer measurement
    z_m.normalize(); // ensure measurement is normalized (unit vector)

    static State::V3 m_world = State::V3(0.23, 0.0, 0.97); // example normalized field
    m_world.normalize();

    const int DoF = iESEKF::Group::Impl::DoF;

    r = iESEKF::Measurement::Zero(3);

    auto R = X_now.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate

    State::V3 z_hat = R.transpose() * m_world; // expected measurement in body frame

    r = z_m - z_hat;

    H = iESEKF::HMat::Zero(3, DoF);
    H.block<3,3>(0, 6) = manif::skew(z_hat); // derivative w.r.t orientation: dm/dθ = -R^t*[m]_x
}

} // namespace ins_ros::iESEKF::magnetometer