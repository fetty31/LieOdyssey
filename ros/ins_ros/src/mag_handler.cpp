#include "ins_ros/mag_handler.hpp"

using namespace ins_ros;

void iESEKF::mag::H_fun(const iESEKF::Filter& kf,
                        const iESEKF::Group& X,
                        iESEKF::Measurement& r,
                        iESEKF::HMat& H)
{
    /* To-Do:
        - get magnetic field measurement 
    */
    State::V3 z_m = State::V3(0.0, 0.0, 0.0); // example magnetometer measurement

    static State::V3 m_world = State::V3(0.23, 0.0, 0.97); // example normalized field

    using Scalar = iESEKF::Scalar;
    
    const int DoF = iESEKF::Group::DoF;

    r = iESEKF::Measurement::Zero(3);

    auto R = X.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate

    State::V3 m_body = R.transpose() * m_world; // expected measurement in body frame

    r = z_m - R.transpose() * m_world;

    H = iESEKF::HMat::Zero(3, DoF);
    H.block<3,3>(0, 6) = manif::skew(m_body); // derivative w.r.t orientation: dm/dθ = -R^t*[m]_x
}