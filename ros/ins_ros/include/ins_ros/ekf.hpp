#pragma once

#include <lie_odyssey/lie_odyssey.hpp>
#include "ins_ros/state.hpp"

namespace ins_ros::iESEKF {

using Scalar = ins_ros::State::Scalar;

using Bundle = lie_odyssey::BundleManif<Scalar, 
                                    manif::SGal3,   // pose + velocity 
                                    manif::R3,      // angular velocity bias
                                    manif::R3,      // acceleration bias
                                    manif::R3       // gravity 
                                    >;

using Group = lie_odyssey::LieGroup<Bundle>;

using Filter  = lie_odyssey::iESEKF<Group>;
using Tangent = Filter::Tangent;
using MatDoF  = Filter::MatDoF;

using Measurement = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
using HMat = Eigen::Matrix<Scalar, Eigen::Dynamic, Bundle::DoF>; // Measurement Jacobian (N measurement x Group DoF)

// Type-conversion helper
void group_to_state(const Group& g, ins_ros::State& state);
void state_to_group(const ins_ros::State& state, Group& g);

// Covariance retrieval (Pose + Vel.)
std::vector<double> get_pose_covariance(const MatDoF& P);
std::vector<double> get_velocity_covariance(const MatDoF& P);

// Propagation model (IMU dynamics)
typename Filter::Tangent f(const Filter& kf, const lie_odyssey::IMUmeas& imu);
typename Filter::Tangent f_state(const ins_ros::State& state);

// Jacobians of the dynamics
typename Filter::Jacobian df_dx(const Filter& kf, const lie_odyssey::IMUmeas& imu);

typename Filter::MappingMatrix df_dw(const Filter& /*kf*/, const lie_odyssey::IMUmeas& /*imu*/);

// Degeneracy handler
void degeneracy_callback(const Filter& /*kf*/, Tangent& dx, const MatDoF& HRH);

} // namespace ins_ros::iESEKF 