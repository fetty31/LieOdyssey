#pragma once

#include "ins_ros/ekf.hpp"
#include <cmath>

namespace ins_ros::iESEKF::barometer {

// Barometer model constants
const iESEKF::Scalar SEA_LEVEL_PRESSURE = 101325.0;   // Pa at sea level
const iESEKF::Scalar TEMPERATURE_LAPLACE = 288.15;   // K (15°C)
const iESEKF::Scalar GAS_CONSTANT = 287.05;          // J/(kg·K) - specific gas constant for dry air
const iESEKF::Scalar GRAVITY = 9.80665;             // m/s^2
const iESEKF::Scalar EXP_COEFFICIENT = 5.2558e-5;   // Specific weight of air (Pa/m)

// International Standard Atmosphere model for pressure at altitude
iESEKF::Scalar pressure_at_altitude(iESEKF::Scalar altitude) {
    // Simplified ISA model for altitudes below ~11km (troposphere)
    // P = P0 * exp(-g * h / (R * T))
    return SEA_LEVEL_PRESSURE * std::exp(-GRAVITY * altitude / (GAS_CONSTANT * TEMPERATURE_LAPLACE));
}

// Derivative of pressure w.r.t altitude (for Jacobian)
iESEKF::Scalar dp_dh(iESEKF::Scalar altitude) {
    return -EXP_COEFFICIENT * pressure_at_altitude(altitude);
}

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, 
            const iESEKF::Group& X_now, 
            const iESEKF::Scalar& y, 
            iESEKF::Measurement& r, 
            iESEKF::HMat& H)
{
    const int DoF = iESEKF::Group::Impl::DoF;

    r = iESEKF::Measurement::Zero(1);

    // Extract position (altitude)
    auto p = X_now.impl().subgroup<0>().translation();
    iESEKF::Scalar z = p.z();

    // Expected pressure based on current altitude
    iESEKF::Scalar p_expected = pressure_at_altitude(z);

    // Residual: difference between expected and actual pressure
    r(0) = y - p_expected; 

    H = iESEKF::HMat::Zero(1, DoF);

    // Jacobian: derivative of residual w.r.t position z
    H(0, 2) = -dp_dh(z);
}

} // namespace ins_ros::iESEKF::barometer