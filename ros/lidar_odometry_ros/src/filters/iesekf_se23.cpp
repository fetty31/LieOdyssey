#include "lidar_odometry_ros/modules/localizer.hpp"
#include "lidar_odometry_ros/modules/mapper.hpp"
#include "lidar_odometry_ros/objects/state.hpp"

#include "lidar_odometry_ros/modules/filters/iesekf_se23.hpp"

using namespace lidar_odometry_ros::iESEKF;

Group lidar_odometry_ros::iESEKF::get_filled_state(const V3& p, const Quat& q, const V3& v, const V3& b_w, const V3& b_a, const V3& gravity)
{
	using NativeBundle = manif::Bundle<iESEKF::Scalar, 
                                            manif::SE_2_3,  // pose + velocity 
                                            manif::R3,     // angular velocity bias
                                            manif::R3,     // acceleration bias
                                            manif::R3      // gravity 
                                            >;
	using SE_2_3 = manif::SE_2_3<iESEKF::Scalar>;
	using R3    = manif::R3<iESEKF::Scalar>;

	auto X0 = NativeBundle(SE_2_3(p,                    // position              0
								  q,                    // orientation           3
								  v),                   // velocity              6      
							R3(b_w), 					// b_w                   9
							R3(b_a), 					// b_a                   12
							R3(gravity)                 // gravity               15
						);  
	auto X = iESEKF::Group(iESEKF::Bundle(X0)); // cast to lie_odyssey type 

	return X;
}

void lidar_odometry_ros::iESEKF::group_to_state(const Group& g, V3& p, Quat& q, V3& v, V3& b_w, V3& b_a, V3& gravity)
{
	iESEKF::Bundle X = g.impl(); 

	// position, orientation, velocity
	p = X.subgroup<0>().translation();
	q = X.subgroup<0>().quat();
	v = X.subgroup<0>().linearVelocity();

	// biases
	b_w = X.subgroup<1>().coeffs();
	b_a = X.subgroup<2>().coeffs();

	// gravity
	gravity = X.subgroup<3>().coeffs();
}

void lidar_odometry_ros::iESEKF::state_to_group(const V3& p, const Quat& q, const V3& v, const V3& b_w, const V3& b_a, const V3& gravity, Group& g)
{
	g = get_filled_state(p, q, v, b_w, b_a, gravity);
}

typename Filter::Tangent lidar_odometry_ros::iESEKF::f(const Filter& kf, const lie_odyssey::IMUmeas& imu) 
{
	// IMU kinematic integration (body-centric):
	// R ⊞ (w - bw - nw)*dt
	// v ⊞ ((a - ba - na) + Rt*g)*dt
	// p ⊞ (v*dt + 1/2*((a - ba - na) + Rt*g)*dt*dt)

	// Build tangent increment xi for SE_2(3) group:
	// xi = [ rho(3); theta(3); nu(3) ] 
    typename Filter::VecTangent t = Filter::VecTangent::Zero();

	Group X = kf.getState(); 
	auto g = X.impl().subgroup<3>().coeffs(); 					// gravity vector estimate
	auto R = X.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate
	auto v0 = X.impl().subgroup<0>().linearVelocity();			// velocity estimate

	// nu (linear acceleration contribution)
	t.template segment<3>(6) = (imu.accel - imu.bias.accel /* -n_a */).cast<Scalar>() - R.transpose() * g;

	// theta (angular velocity contribution)
	t.template segment<3>(3) = (imu.gyro - imu.bias.gyro /* -n_w */).cast<Scalar>();

	// rho (position): 
	t.template segment<3>(0) = R.transpose()*v0 + 0.5 * t.template segment<3>(6) * imu.dt;

    return t; // cast to Tangent
}

typename Filter::Tangent lidar_odometry_ros::iESEKF::f_state(const Group& g, const lie_odyssey::IMUmeas& imu) 
{
	// Build tangent increment xi for SE_2(3) group:
	// xi = [ rho(3); theta(3); nu(3) ] 
    typename Filter::VecTangent t = Filter::VecTangent::Zero();

	auto grav = g.impl().subgroup<3>().coeffs(); 				// gravity vector estimate
	auto R = g.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate
	auto v0 = g.impl().subgroup<0>().linearVelocity();			// velocity estimate

	// nu (linear acceleration contribution)
	t.template segment<3>(6) = (imu.accel - imu.bias.accel /* -n_a */).cast<Scalar>() - R.transpose() * grav;

	// theta (angular velocity contribution)
	t.template segment<3>(3) = (imu.gyro - imu.bias.gyro /* -n_w */).cast<Scalar>();

	// rho (position): 
	t.template segment<3>(0) =  R.transpose()*v0 + 0.5 * t.template segment<3>(6) * imu.dt;

    return t; // cast to Tangent
}

typename Filter::Jacobian lidar_odometry_ros::iESEKF::df_dx(const Filter& kf, const lie_odyssey::IMUmeas& imu) 
{
	// IMU kinematic integration (body-centric):
	// R ⊞ (w - bw - nw)*dt
	// v ⊞ ((a - ba - na) + Rt*g)*dt
	// p ⊞ (v*dt + 1/2*((a - ba - na) + Rt*g)*dt*dt)

    Filter::Jacobian Jx = Filter::Jacobian::Zero();

	Group X = kf.getState(); 
	auto g = X.impl().subgroup<3>().coeffs(); 					// gravity estimate
	auto R = X.impl().subgroup<0>().quat().toRotationMatrix();	// orientation estimate
	auto v0 = X.impl().subgroup<0>().linearVelocity();			// velocity estimate

	// position
	Jx.block<3, 3>(0, 3) = manif::skew(R.transpose()*v0)          				// w.r.t R := d(R^-1*g)/dR * d(R^-1)/dR
						- manif::skew(R.transpose()*g)*imu.dt*0.5;				//            + d(R^-1*v)/dR * d(R^-1)/dR
	Jx.block<3, 3>(0, 6)  = R.transpose();										// w.r.t v
	Jx.block<3, 3>(0, 12) = -Eigen::Matrix<Scalar,3,3>::Identity()*0.5*imu.dt; 	// w.r.t b_a
	Jx.block<3, 3>(0, 15) = -R.transpose()*0.5*imu.dt; 							// w.r.t g

	// velocity 
    Jx.block<3, 3>(6,  3) = -manif::skew(R.transpose() * g);	        // w.r.t R := d(R^t*g)/dR 
    Jx.block<3, 3>(6, 12) = -Eigen::Matrix<Scalar,3,3>::Identity();     // w.r.t b_a 
    Jx.block<3, 3>(6, 15) = -R.transpose(); 				 	 		// w.r.t g

    // rotation
    Jx.block<3, 3>(3, 9) = -Eigen::Matrix<Scalar,3,3>::Identity();     // w.r.t b_w

    return Jx;
}

typename Filter::MappingMatrix lidar_odometry_ros::iESEKF::df_dw(const Filter& /*kf*/, const lie_odyssey::IMUmeas& imu) 
{
    // w = (n_w, n_a, n_{b_w}, n_{b_a})
    Filter::MappingMatrix Jw = Filter::MappingMatrix::Zero();

	// position
	Jw.block<3, 3>(0, 3)  = -Eigen::Matrix<Scalar,3,3>::Identity()*0.5*imu.dt; // w.r.t n_a
	// velocity
    Jw.block<3, 3>(6, 3)  = -Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_a
	// rotation
    Jw.block<3, 3>(3, 0)  = -Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_w
	// bias
    Jw.block<3, 3>(9, 6)  =  Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_{b_w}
    Jw.block<3, 3>(12, 9) =  Eigen::Matrix<Scalar,3,3>::Identity(); // w.r.t n_{b_a}

    return Jw;
}

void lidar_odometry_ros::iESEKF::H_fun(const Filter& /*kf*/, const Group& X_now, Measurement& z, HMat& H)
{
    lidar_odometry_ros::Localizer& LOC = lidar_odometry_ros::Localizer::getInstance();
	lidar_odometry_ros::Mapper& MAP = lidar_odometry_ros::Mapper::getInstance();

	// Calculate matches
	// (maybe better to freeze last state: X_now -> LOC.last_state)
	Matches matches = MAP.match(
	    lidar_odometry_ros::State (X_now),
	    LOC.pc2match
	);

	// // Calculate derivatives
	LOC.calculate_H(
	    // Inputs
	    X_now,
	    matches,

	    // Outputs (z := residual vector, H := measurement jacobian dh/dx)
	    z,
	    H
	);
}

// Fill Measurement Jacobian H row with point-to-plane residual derivative 
void lidar_odometry_ros::iESEKF::fill_H_point_to_plane(const Group& group, 
														const V3& normal, 
														const V3& point, 
														int i, 
														HMat& H)
{
	iESEKF::Bundle s = group.impl(); // lie_odyssey::ManifBundle object
	manif::SE_2_3<Scalar> SE23 = s.subgroup<0>();

	// Compute jacobian w.r.t. state
	Eigen::Matrix<Scalar, 3, manif::SE_2_3<Scalar>::DoF> J_dX; // jacobian SE_2_3 action := J_dX ​= d(G * p_imu)/dX​
	SE23.act(point, J_dX);

	// Fill H with state part
	H.block<1, manif::SE_2_3<Scalar>::DoF>(i, 0) = (normal.transpose() * J_dX).eval();
}

void lidar_odometry_ros::iESEKF::degeneracy_callback(const Filter& /*kf*/, Tangent& dx, const MatDoF& HRH)
{
	/* 
		To-Do: handle degeneracy in SE_2(3) group (first subgroup of our Bundle state)
	*/
}