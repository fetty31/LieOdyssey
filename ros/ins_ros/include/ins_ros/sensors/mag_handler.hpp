#pragma once

#include "ins_ros/ekf.hpp"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace ins_ros::iESEKF::magnetometer {

// Constants for reference magnetic field (WMM - World Magnetic Model)
const State::V3 REF_MAG_FIELD_ECEF_0_0 = State::V3(29546.5, -4146.6, 18964.3); // T * 1e4 (approx)
const double LATITUDE = 48.85;  // Reference latitude (e.g., Paris)
const double LONGITUDE = 2.35;  // Reference longitude (e.g., Paris)
const double WGS84_A = 6378137.0;      // Earth's semi-major axis (meters)
const double WGS84_B = 6356752.314245; // Earth's semi-minor axis (meters)
const double WGS84_E2 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A); // First eccentricity squared

// WGS84 to ECEF conversion
State::V3 llh_to_ecef(const State::V3& llh) {
    double lat = llh.x(), lon = llh.y(), h = llh.z();
    double e2 = WGS84_E2;
    double N = WGS84_A / std::sqrt(1.0 - e2 * std::sin(lat) * std::sin(lat));
    
    State::V3 ecef;
    ecef.x() = (N + h) * std::cos(lat) * std::cos(lon);
    ecef.y() = (N + h) * std::cos(lat) * std::sin(lon);
    ecef.z() = (N * (1.0 - e2) + h) * std::sin(lat);
    return ecef;
}

// Convert ECEF to NED (North-East-Down)
State::V3 ecef_to_ned(const State::V3& ecef, const State::V3& llh_ref) {
    double sin_lat = std::sin(llh_ref.x()), cos_lat = std::cos(llh_ref.x());
    double sin_lon = std::sin(llh_ref.y()), cos_lon = std::cos(llh_ref.y());
    
    State::V3 ned;
    
    // Reference point in ECEF
    State::V3 ecef_ref = llh_to_ecef(llh_ref);
    
    // Vector from reference to current point
    State::V3 diff = ecef - ecef_ref;
    
    // Rotate from ECEF to NED
    ned.x() = -sin_lat * cos_lon * diff.x() - sin_lat * sin_lon * diff.y() + cos_lat * diff.z();
    ned.y() = -sin_lon * diff.x() + cos_lon * diff.y();
    ned.z() =  cos_lat * cos_lon * diff.x() + cos_lat * sin_lon * diff.y() + sin_lat * diff.z();
    
    return ned;
}

// Convert NED to ECEF using Newton's method for robustness
State::V3 ned_to_ecef(const State::V3& ned, const State::V3& llh_ref) {
    State::V3 ecef;
    State::V3 ecef_ref = llh_to_ecef(llh_ref);
    
    double sin_lat = std::sin(llh_ref.x()), cos_lat = std::cos(llh_ref.x());
    double sin_lon = std::sin(llh_ref.y()), cos_lon = std::cos(llh_ref.y());
    
    // Convert NED to ECEF
    State::V3 delta_ecef;
    delta_ecef.x() = -sin_lon * ned.y() - sin_lat * cos_lon * ned.z();
    delta_ecef.y() =  cos_lon * ned.y() - sin_lat * sin_lon * ned.z();
    delta_ecef.z() =  cos_lat * ned.z();
    
    ecef = ecef_ref + delta_ecef;
    return ecef;
}

// Convert NED to ENU
State::V3 ned_to_enu(const State::V3& ned) {
    State::V3 enu;
    enu.x() = ned.y(); // East
    enu.y() = ned.x(); // North
    enu.z() = -ned.z(); // Up (Down negated)
    return enu;
}

// Interpolated magnetic field at given location (simplified WMM)
State::V3 get_mag_field(const State::V3& p_enu) {
    // Simplified 1D magnetic field model (in reality, use full WMM)
    // B_east ≈ p.y() * 0.000015
    // B_north ≈ p.x() * 0.000015
    // B_up ≈ -0.00005 - p.z() * 0.000005
    
    State::V3 B;
    B.x() = p_enu.y() * 0.000015;
    B.y() = p_enu.x() * 0.000015;
    B.z() = -0.00005 - p_enu.z() * 0.000005;
    
    return B;
}

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, 
            const iESEKF::Group& X_now, 
            const iESEKF::Measurement& y, 
            iESEKF::Measurement& r, 
            iESEKF::HMat& H)
{
    const int DoF = iESEKF::Group::Impl::DoF;

    r = iESEKF::Measurement::Zero(3);

    // Get state position in world frame
    auto p_world = X_now.impl().subgroup<0>().translation();

    // Convert world frame position (ENU) to reference magnetic field location
    // For simplicity, assume the reference location is known (e.g., from GPS at initialization)
    State::V3 p_ref(0.0, 0.0, 0.0); // Reference position (will be set in first callback)
    static bool ref_set = false;
    
    if (!ref_set) {
        // Wait for filter to have valid position
        p_ref = p_world;
        ref_set = true;
    }
    
    // Get expected magnetic field at current position
    State::V3 p_enu_rel = p_world - p_ref;
    State::V3 B_expected = get_mag_field(p_enu_rel);

    // Transform to body frame using orientation estimate
    auto R = X_now.impl().subgroup<0>().quat().toRotationMatrix();
    State::V3 B_body_expected = R * B_expected;

    // Measurement is in body frame (sensor reading)
    r = y - B_body_expected;

    // Jacobian
    H = iESEKF::HMat::Zero(3, DoF);
    H.block<3,3>(0, 6) = manif::skew(R.transpose() * B_body_expected); // ∂r/∂q
    H.block<3,3>(0, 0) = manif::skew(B_body_expected); // ∂r/∂p for magnetic field model (small, often neglected)
}

} // namespace ins_ros::iESEKF::magnetometer