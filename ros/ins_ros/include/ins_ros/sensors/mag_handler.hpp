#pragma once

#include "ins_ros/ekf.hpp"

namespace ins_ros::iESEKF::magnetometer {

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