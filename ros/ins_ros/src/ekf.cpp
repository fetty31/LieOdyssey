#include "ins_ros/ekf.hpp"

using namespace ins_ros::iESEKF;

void ins_ros::iESEKF::group_to_state(const Group& g, ins_ros::State& state)
{
	iESEKF::Bundle X = g.impl(); 

	// position, orientation, velocity
	state.p = X.subgroup<0>().translation();
	state.q = X.subgroup<0>().quat();
	state.v = X.subgroup<0>().linearVelocity();

	// biases
	state.bias.w = X.subgroup<1>().coeffs();
	state.bias.a = X.subgroup<2>().coeffs();

	// gravity
	state.g = X.subgroup<3>().coeffs();
}

void ins_ros::iESEKF::state_to_group(const ins_ros::State& state, Group& g)
{
    using NativeBundle = manif::Bundle<iESEKF::Scalar, 
                                            manif::SGal3,  // pose + velocity 
                                            manif::R3,      // angular velocity bias
                                            manif::R3,      // acceleration bias
                                            manif::R3       // gravity 
                                            >;
	using SGal3 = manif::SGal3<iESEKF::Scalar>;
	using R3    = manif::R3<iESEKF::Scalar>;

	auto X0 = NativeBundle(SGal3(state.p,                    // x y z                  0
								state.q,                     // rotation               6
								state.v,                     // vx, vy, vz             3
								0.),                         // delta t                9
							R3(state.bias.w), 				 // b_w                   10
							R3(state.bias.a), 				 // b_a                   13
							R3(state.g)                      // gravity               16
						);  
	g = iESEKF::Group(iESEKF::Bundle(X0)); // cast to lie_odyssey type 
}

std::vector<double> ins_ros::iESEKF::get_pose_covariance(const MatDoF& P)
{
    Eigen::Matrix<double, 6, 6> P_pose;
    P_pose.block<3, 3>(0, 0) = P.block<3, 3>(0, 0).cast<double>();
    P_pose.block<3, 3>(0, 3) = P.block<3, 3>(0, 6).cast<double>();
    P_pose.block<3, 3>(3, 0) = P.block<3, 3>(6, 0).cast<double>();
    P_pose.block<3, 3>(3, 3) = P.block<3, 3>(6, 6).cast<double>();

    std::vector<double> cov(P_pose.size());
    Eigen::Map<Eigen::MatrixXd>(cov.data(), P_pose.rows(), P_pose.cols()) = P_pose;
    return cov;
}

std::vector<double> ins_ros::iESEKF::get_velocity_covariance(const MatDoF& P)
{
    Eigen::Matrix<double, 6, 6> P_odom = Eigen::Matrix<double, 6, 6>::Zero();
    P_odom.block<3, 3>(0, 0) = P.block<3, 3>(3, 3).cast<double>();

    std::vector<double> cov(P_odom.size());
    Eigen::Map<Eigen::MatrixXd>(cov.data(), P_odom.rows(), P_odom.cols()) = P_odom;
    return cov;
}

typename Filter::Tangent ins_ros::iESEKF::f(const Filter& kf, const lie_odyssey::IMUmeas& imu) 
{
	// IMU kinematic integration (body-centric):
	// R ⊞ (w - bw - nw)*dt
	// v ⊞ ((a - ba - na) + Rt*g)*dt
	// p ⊞ (v*dt + 1/2*((a - ba - na) + Rt*g)*dt*dt)

	// Build tangent increment xi for SGal(3) group:
	// xi = [ rho(3); nu(3); theta(3); s(1) ] 
    typename Filter::VecTangent t = Filter::VecTangent::Zero();

	Group X = kf.getState(); 
	auto g = X.impl().subgroup<3>().coeffs(); 					// gravity vector estimate
	auto R = X.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate

	// rho (position): zero

	// nu (linear acceleration contribution)
	t.template segment<3>(3) = (imu.accel - imu.bias.accel /* -n_a */).cast<Scalar>() - R.transpose() * g;

	// theta (angular velocity contribution)
	t.template segment<3>(6) = (imu.gyro - imu.bias.gyro /* -n_w */).cast<Scalar>();

	// s (time)
	t(9) = Scalar(1);

    return t; // cast to Tangent
}

typename Filter::Tangent ins_ros::iESEKF::f_state(const ins_ros::State& state) 
{
	// Build tangent increment xi for SGal(3) group:
	// xi = [ rho(3); nu(3); theta(3); s(1) ] 
    typename Filter::VecTangent t = Filter::VecTangent::Zero();

	// nu (linear acceleration contribution)
	t.template segment<3>(3) = (state.a - state.bias.a /* -n_a */) - R.transpose() * state.g;

	// theta (angular velocity contribution)
	t.template segment<3>(6) = (state.w - state.bias.w /* -n_w */);

	// rho (position): 
	t.template segment<3>(0) = state.v; // p ⊞ v*dt

	// s (time)
	t(9) = Scalar(1);

    return t; // cast to Tangent
}

typename Filter::Jacobian ins_ros::iESEKF::df_dx(const Filter& kf, const lie_odyssey::IMUmeas& /*imu*/) 
{
	// IMU kinematic integration (body-centric):
	// R ⊞ (w - bw - nw)*dt
	// v ⊞ ((a - ba - na) + Rt*g)*dt
	// p ⊞ (v*dt + 1/2*((a - ba - na) + Rt*g)*dt*dt)

    Filter::Jacobian Jx = Filter::Jacobian::Zero();

	Group X = kf.getState(); 
	auto g = X.impl().subgroup<3>().coeffs(); 					// gravity estimate
	auto R = X.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate

	// velocity 
    Jx.block<3, 3>(3,  6) = -manif::skew(R.transpose() * g);	        // w.r.t R := d(R^t*g)/dR 
    Jx.block<3, 3>(3, 13) = -Eigen::Matrix<Scalar,3,3>::Identity();     // w.r.t b_a 
    Jx.block<3, 3>(3, 16) = -R.transpose(); 				 	 		// w.r.t g

    // rotation
    Jx.block<3, 3>(6, 10) = -Eigen::Matrix<Scalar,3,3>::Identity();     // w.r.t b_w

    return Jx;
}

typename Filter::MappingMatrix ins_ros::iESEKF::df_dw(const Filter& /*kf*/, const lie_odyssey::IMUmeas& /*imu*/) 
{
    // w = (n_w, n_a, n_{b_w}, n_{b_a})
    Filter::MappingMatrix Jw = Filter::MappingMatrix::Zero();

    Jw.block<3, 3>(3, 3)  = -Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_a
    Jw.block<3, 3>(6, 0)  = -Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_w
    Jw.block<3, 3>(10, 6) =  Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_{b_w}
    Jw.block<3, 3>(13, 9) =  Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_{b_a}
    
    return Jw;
}

void ins_ros::iESEKF::degeneracy_callback(const Filter& /*kf*/, Tangent& /*dx*/, const MatDoF& /*HRH*/)
{
	/* 
		To-Do: handle degeneracy in SGal3 group (first subgroup of our Bundle state)
	*/
}