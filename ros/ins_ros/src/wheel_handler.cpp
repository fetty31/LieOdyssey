#include "ins_ros/wheel_handler.hpp"

using namespace ins_ros;

void iESEKF::wheel::H_fun(const iESEKF::Filter& kf,
                        const iESEKF::Group& X,
                        iESEKF::Measurement& r,
                        iESEKF::HMat& H)
{
    /* To-Do:
        - get wheel odometry measurement in filter frame (e.g., ENU)
    */
    static State::V3 wheel_measurement = State::V3(0.0, 0.0, 0.0); // example wheel odometry measurement

    using Scalar = iESEKF::Scalar;
    
    const int DoF = iESEKF::Group::DoF;

    r = iESEKF::Measurement::Zero(3);

    auto R = X.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate
    auto v = X.subgroup<0>().linearVelocity();                  // velocity estimate

    auto v_body = R.transpose() * v; // expected measurement in body frame

    r = wheel_measurement - v_body;

    H = iESEKF::HMat::Zero(3, DoF);

    // velocity part
    H.block<3,3>(0, 3) = R.transpose();

    // orientation coupling: δθ × v
    H.block<3,3>(0, 6) = manif::skew(v_body);
}