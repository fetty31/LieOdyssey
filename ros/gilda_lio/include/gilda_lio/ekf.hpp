#pragma once

#include <lie_odyssey/lie_odyssey.hpp>
#include "gilda_lio/state.hpp"

namespace gilda_lio::iESEKF {

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

// Type-conversion helper
void group_to_state(const Group& g, gilda_lio::State& state);
void state_to_group(const gilda_lio::State& state, Group& g);

// Covariance retrieval (Pose + Vel.)
std::vector<double> get_pose_covariance(const MatDoF& P);
std::vector<double> get_velocity_covariance(const MatDoF& P);

// Propagation model (IMU dynamics)
typename Filter::Tangent f(const Filter& kf, const lie_odyssey::IMUmeas& imu);
typename Filter::Tangent f_state(const gilda_lio::State& state);

// Jacobians of the dynamics
typename Filter::Jacobian df_dx(const Filter& kf, const lie_odyssey::IMUmeas& imu);

typename Filter::MappingMatrix df_dw(const Filter& /*kf*/, const lie_odyssey::IMUmeas& /*imu*/);

// Measurement function
void H_fun(const Filter& /*kf*/, const Group& X_now, Measurement& z, HMat& H);

// Fill Measurement Jacobian H row with point-to-plane residual derivative 
void fill_H_point_to_plane(const Group& group, const V3& normal, const V3& point, int i, HMat& H);

// Degeneracy handler
void degeneracy_callback(const Filter& /*kf*/, Tangent& dx, const MatDoF& HRH);

} // namespace gilda_lio::iESEKF 