#pragma once

#include <lie_odyssey/lie_odyssey.hpp>

namespace lidar_odometry_ros::iESEKF {

using Scalar = double;

using V3 = Eigen::Matrix<Scalar, 3, 1>;
using Quat = Eigen::Quaternion<Scalar>;

using Bundle = lie_odyssey::BundleManif<Scalar, 
                                    manif::SGal3,  // pose + velocity 
                                    manif::R3,     // angular velocity bias
                                    manif::R3,     // acceleration bias
                                    manif::R3      // gravity (To-Do make it S2 group)
                                    >;

using Group = lie_odyssey::LieGroup<Bundle>;

using Filter  = lie_odyssey::iESEKF<Group>;
using Tangent = Filter::Tangent;
using MatDoF  = Filter::MatDoF;

using Measurement = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using HMat = Eigen::Matrix<Scalar, Eigen::Dynamic, Bundle::DoF>; // Measurement Jacobian (N measurement x Group DoF)

// Initialization
Group get_filled_state(const V3& p, const Quat& q, const V3& v, const V3& b_w, const V3& b_a, const V3& gravity);

// Type-conversion helper
void group_to_state(const Group& g, V3& p, Quat& q, V3& v, V3& b_w, V3& b_a, V3& gravity);
void state_to_group(const V3& p, const Quat& q, const V3& v, const V3& b_w, const V3& b_a, const V3& gravity, Group& g);

// Propagation model (IMU dynamics)
typename Filter::Tangent f(const Filter& kf, const lie_odyssey::IMUmeas& imu);
typename Filter::Tangent f_state(const Group& g, const lie_odyssey::IMUmeas& imu);

// Jacobians of the dynamics
typename Filter::Jacobian df_dx(const Filter& kf, const lie_odyssey::IMUmeas& imu);

typename Filter::MappingMatrix df_dw(const Filter& /*kf*/, const lie_odyssey::IMUmeas& /*imu*/);

// Measurement function
void H_fun(const Filter& /*kf*/, const Group& X_now, Measurement& z, HMat& H);

// Fill Measurement Jacobian H row with point-to-plane residual derivative 
void fill_H_point_to_plane(const Group& group, const V3& normal, const V3& point, int i, HMat& H);

// Degeneracy handler
void degeneracy_callback(const Filter& /*kf*/, Tangent& dx, const MatDoF& HRH);

} // namespace lidar_odometry_ros::iESEKF 