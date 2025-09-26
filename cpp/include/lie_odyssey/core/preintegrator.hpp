#ifndef __LIEODYSSEY_CORE_PREINTEGRATOR_HPP__
#define __LIEODYSSEY_CORE_PREINTEGRATOR_HPP__

#include "lie_odyssey/core/groups.hpp"
#include "lie_odyssey/core/imu_data.hpp"
#include "lie_odyssey/core/preintegrator_traits.hpp"

namespace lie_odyssey {

template <typename Group>
class Preintegrator {

 public:
    using Scalar  = typename Group::Impl::Native::Scalar;
    using Tangent = typename Group::Tangent;            
    using Jacob   = typename Group::Jacobian;           
    using Matrix  = typename Group::MatrixType; 

    static constexpr int DoF = Group::Impl::DoF;

    using Mat3    = Eigen::Matrix<Scalar,3,3>;
    using Vec3    = Eigen::Matrix<Scalar,3,1>;

    using VecTangent = Eigen::Matrix<Scalar,DoF,1>;
    using MatNoise   = Eigen::Matrix<Scalar,DoF,6>;
    using Mat6       = Eigen::Matrix<Scalar,6,6>;

    Preintegrator(const Mat3& cova = Mat3::Identity() * Scalar(1e-3),
                  const Mat3& covg = Mat3::Identity() * Scalar(1e-5),
                  const Vec3& gravity = Vec3(Scalar(0.0), Scalar(0.0), Scalar(-9.81)))
        : dX(),
            J_bg(Eigen::Matrix<Scalar,DoF,3>::Zero()),
            J_ba(Eigen::Matrix<Scalar,DoF,3>::Zero()),
            cov(Jacob::Zero()),
            cov_gyro(covg),
            cov_acc(cova),
            gravity(gravity),
            sum_dt(Scalar(0))
    {}

    void reset() 
    {
        dX.setIdentity();
        J_bg.setZero();
        J_ba.setZero();
        cov.setZero();
        sum_dt = Scalar(0);
    }

    // Integrate one IMU sample.
    // acc_meas, omega_meas: raw IMU measurements (m/s^2, rad/s)
    // ba_nominal, bg_nominal: current bias estimates (m/s^2, rad/s)
    // dt: timestep (s)
    void integrate( const Vec3& acc_meas,
                    const Vec3& omega_meas,
                    const Vec3& ba_nominal,
                    const Vec3& bg_nominal,
                    Scalar dt)
    {
        // unbiased measurements (nominally corrected; measurement noise still present)
        Vec3 omega_unbiased = omega_meas - bg_nominal;
        Vec3 acc_unbiased   = acc_meas  - ba_nominal;

        // Transform body frame acceleration to inertial frame
        Vec3 acc_global = dX.impl().R() * acc_unbiased + gravity;

        // Build tangent increment xi:
        Tangent xi = PreintegrationTraits<Group>::get_tangent(acc_global, 
                                                            omega_unbiased, 
                                                            dt, dX);

        // Compose: dX_new = dX ⊕ exp(xi)  i.e. right-plus: dX.plus(xi)
        Jacob J_dX;   // ∂(dX ⊕ exp(xi)) / ∂dX  == Adj(exp(xi))^-1
        Jacob J_xi;   // ∂(dX ⊕ exp(xi)) / ∂xi  == Jr
        dX.plus(xi, J_dX, J_xi);

        // --- compute ∂xi/∂b_g and ∂xi/∂b_a ---
        Eigen::Matrix<Scalar,DoF,3> dxi_dbg = Eigen::Matrix<Scalar,DoF,3>::Zero();
        Eigen::Matrix<Scalar,DoF,3> dxi_dba = Eigen::Matrix<Scalar,DoF,3>::Zero();
        PreintegrationTraits<Group>::get_dX_dbias(dxi_dba, dxi_dbg, dt);

        // now propagate bias Jacobians by chain rule:
        // J_bg_new = J_dX * J_bg_old + J_xi * dxi_dbg
        // J_ba_new = J_dX * J_ba_old + J_xi * dxi_dba
        Eigen::Matrix<Scalar,DoF,3> J_bg_new = J_dX * J_bg + J_xi * dxi_dbg;
        Eigen::Matrix<Scalar,DoF,3> J_ba_new = J_dX * J_ba + J_xi * dxi_dba;

        // --- covariance propagation ---
        // measurement noise vector n = [n_g(3); n_a(3)], discrete covariance Qm = diag(cov_gyro*dt, cov_acc*dt)
        Mat6 Qm = Mat6::Zero();
        Qm.template block<3,3>(0,0) = cov_gyro * dt;
        Qm.template block<3,3>(3,3) = cov_acc  * dt;

        // map measurement noise n -> tangent xi via G (DoFx6)
        MatNoise G = MatNoise::Zero();
        PreintegrationTraits<Group>::get_meas_noise(G, dt);

        // discrete process noise mapped to tangent: J_xi * G * Qm * G^T * J_xi^T
        Jacob process_noise = J_xi * (G * Qm * G.transpose()) * J_xi.transpose();

        // covariance update: P_new = J_dX * P * J_dX^T + process_noise
        Jacob cov_new = J_dX * cov * J_dX.transpose() + process_noise;

        // commit updates
        J_bg = J_bg_new;
        J_ba = J_ba_new;
        cov = cov_new;
        sum_dt += dt;
    }

    // Integrate one IMU sample.
    // imu_meas: raw IMU measurements (rad/s, m/s^2)
    //           including current bias estimates (rad/s, m/s^2)
    //           as well as timestep (s)
    void integrate(const IMUmeas& imu_meas)
    {
        integrate(imu_meas.accel, imu_meas.gyro,
                  imu_meas.bias.accel, imu_meas.bias.gyro, 
                  imu_meas.dt
                );
    }

    // Apply bias correction: given small bias deltas (delta_ba, delta_bg)
    void correctDelta(const Vec3& delta_ba, const Vec3& delta_bg)
    {
        // correction in tangent: corr = J_bg * delta_bg + J_ba * delta_ba (DoFx1)
        auto corrT = PreintegrationTraits<Group>::get_correction(J_ba, J_bg, delta_ba, delta_bg);
        dX.plus(corrT); // apply correction on right: dX_corr = dX ⊕ Exp(corr)
    }

    // Apply bias correction: given small bias deltas (delta_ba, delta_bg) return corrected dX
    Group getAndCorrectDelta(const Vec3& delta_ba, const Vec3& delta_bg) {
        correctDelta(delta_ba, delta_bg);
        return dX;
    }

    // Group accessor
    Group getState() { return dX; }

    // Time accessor
    Scalar getTotalTime() { return sum_dt; }

    // Update current gravity estimate
    void updateGravity(const Vec3& g) { gravity = g; }

 protected:

    // Preintegrated group element (Δ in Lie Group)
    Group dX;

    // Jacobians of preintegrated Δ wrt biases:
    //  J_bg: ∂(log Δ)/∂b_g (DoFx3)  (in tangent coordinates)
    //  J_ba: ∂(log Δ)/∂b_a (DoFx3)
    Eigen::Matrix<Scalar,DoF,3> J_bg;
    Eigen::Matrix<Scalar,DoF,3> J_ba;

    // Covariance on the tangent space (Δ tangent uncertainty)
    Jacob cov;

    // Gravity vector
    Vec3 gravity;

    // IMU continuous noise spectral densities (per second)
    Mat3 cov_gyro; // (rad/s)^2 / Hz
    Mat3 cov_acc;  // (m/s^2)^2 / Hz

    Scalar sum_dt = Scalar(0);
};

} // namespace lie_odyssey

#endif // __LIEODYSSEY_CORE_PREINTEGRATOR_HPP__