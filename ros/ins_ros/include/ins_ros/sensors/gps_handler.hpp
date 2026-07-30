#pragma once

#include "ins_ros/ekf.hpp"
#include <cmath>

namespace ins_ros::iESEKF::gps {

// Constants for coordinate conversion
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

// Convert NED (North-East-Down) to ENU (East-North-Up)
State::V3 ned_to_enu(const State::V3& ned) {
    State::V3 enu;
    enu.x() = ned.y(); // East
    enu.y() = ned.x(); // North
    enu.z() = -ned.z(); // Up (negate Down)
    return enu;
}

// ENU to NED conversion
State::V3 enu_to_ned(const State::V3& enu) {
    State::V3 ned;
    ned.x() = enu.y(); // North
    ned.y() = enu.x(); // East
    ned.z() = -enu.z(); // Down
    return ned;
}

// Local tangent plane projection: LLA to ENU (flat Earth approximation)
State::V3 lla_to_enu(const State::V3& llh, const State::V3& llh_origin) {
    State::V3 enu;
    
    // Convert degrees to radians
    double lat0 = llh_origin.x() * M_PI / 180.0;
    
    // Earth radius at latitude
    double R = WGS84_A;
    
    // North and East displacements using spherical approximation
    enu.x() = (llh.y() - llh_origin.y()) * M_PI / 180.0 * R * std::cos(lat0); // East
    enu.y() = (llh.x() - llh_origin.x()) * M_PI / 180.0 * R;                  // North
    enu.z() = llh.z() - llh_origin.z();                                         // Up
    
    return enu;
}

// ECEF to ENU conversion (relative to origin at llh reference)
State::V3 ecef_to_enu(const State::V3& ecef, const State::V3& llh_origin) {
    State::V3 enu;
    State::V3 ecef_origin = llh_to_ecef(llh_origin);
    
    State::V3 diff = ecef - ecef_origin;
    
    // Convert ECEF difference to ENU
    double lat = llh_origin.x() * M_PI / 180.0;
    double lon = llh_origin.y() * M_PI / 180.0;
    
    enu.x() = -std::sin(lon) * diff.x() + std::cos(lon) * diff.y();
    enu.y() = -std::sin(lat) * std::cos(lon) * diff.x() - std::sin(lat) * std::sin(lon) * diff.y() + std::cos(lat) * diff.z();
    enu.z() =  std::cos(lat) * std::cos(lon) * diff.x() + std::cos(lat) * std::sin(lon) * diff.y() + std::sin(lat) * diff.z();
    
    return enu;
}

// ENU to ECEF (simplified inverse)
State::V3 enu_to_ecef(const State::V3& enu, const State::V3& llh_origin) {
    State::V3 ecef;
    State::V3 ecef_origin = llh_to_ecef(llh_origin);
    
    double lat = llh_origin.x() * M_PI / 180.0;
    double lon = llh_origin.y() * M_PI / 180.0;
    
    // Convert ENU to ECEF difference
    double x = std::cos(lon) * enu.x() - std::sin(lon) * enu.y();
    double y = -std::sin(lat) * std::cos(lon) * enu.x() - std::sin(lat) * std::sin(lon) * enu.y() + std::cos(lat) * enu.z();
    double z =  std::cos(lat) * std::cos(lon) * enu.x() + std::cos(lat) * std::sin(lon) * enu.y() + std::sin(lat) * enu.z();
    
    ecef.x() = ecef_origin.x() + x;
    ecef.y() = ecef_origin.y() + y;
    ecef.z() = ecef_origin.z() + z;
    
    return ecef;
}

// Measurement function
void H_fun(const iESEKF::Filter& /*kf*/, 
            const iESEKF::Group& X_now, 
            const iESEKF::Measurement& y, 
            iESEKF::Measurement& r, 
            iESEKF::HMat& H)
{
    using Scalar = iESEKF::Scalar;
    
    const int DoF = iESEKF::Group::Impl::DoF;

    // Measurement: position only (3D GPS fix in ENU)
    r = iESEKF::Measurement::Zero(3);

    // Extract position estimate in world frame
    State::V3 p_hat = X_now.impl().subgroup<0>().translation();
    
    // Assume measurement is already converted to ENU frame
    State::V3 y_enu = y.head<3>().cast<Scalar>();

    r.segment<3>(0) = y_enu - p_hat;

    // Jacobian
    H = iESEKF::HMat::Zero(3, DoF);

    // dp/dp = Identity
    H.block<3,3>(0, 0) = Eigen::Matrix<Scalar, 3, 3>::Identity(); 
}

} // namespace ins_ros::iESEKF::gps