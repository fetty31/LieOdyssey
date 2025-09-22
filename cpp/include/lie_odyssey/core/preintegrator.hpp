#ifndef __LIEODYSSEY_CORE_PREINTEGRATOR_HPP__
#define __LIEODYSSEY_CORE_PREINTEGRATOR_HPP__

#include "lie_odyssey/core/groups.hpp"
#include "lie_odyssey/core/imu_data.hpp"

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

    Preintegrator(const Mat3& covg = Mat3::Identity() * Scalar(1e-5),
                  const Mat3& cova = Mat3::Identity() * Scalar(1e-3))
        : dX(),
            J_bg(Eigen::Matrix<Scalar,DoF,3>::Zero()),
            J_ba(Eigen::Matrix<Scalar,DoF,3>::Zero()),
            cov(Jacob::Zero()),
            cov_gyro(covg),
            cov_acc(cova),
            sum_dt(Scalar(0))
    {}

    virtual void reset() 
    {
        dX.setIdentity();
        J_bg.setZero();
        J_ba.setZero();
        cov.setZero();
        sum_dt = Scalar(0);
    }

    // Integrate one IMU sample.
    // omega_meas, acc_meas: raw IMU measurements (rad/s, m/s^2)
    // bg_nominal, ba_nominal: current bias estimates (rad/s, m/s^2)
    // dt: timestep (s)
    virtual void integrate(const Vec3& omega_meas,
                    const Vec3& acc_meas,
                    const Vec3& bg_nominal,
                    const Vec3& ba_nominal,
                    Scalar dt)
    {
        // unbiased measurements (nominally corrected; measurement noise still present)
        Vec3 omega_unbiased = omega_meas - bg_nominal;
        Vec3 acc_unbiased   = acc_meas  - ba_nominal;

        // Build tangent increment xi:
        // xi = [ rho(3); nu(3); theta(3); s(1) ] for SGal(3)
        VecTangent tangent = VecTangent::Zero();

        // nu (acc contribution)
        tangent.template segment<3>(3) = acc_unbiased * dt;

        // theta (rotation)
        tangent.template segment<3>(6) = omega_unbiased * dt;

        // s (time)
        tangent(9) = dt;

        // rho (position): approximate displacement contribution over this small step
        // use current preintegrated delta-velocity
        Vec3 cur_dv = dX.impl().v(); 
        tangent.template segment<3>(0) = cur_dv * dt + Scalar(0.5) * acc_unbiased * dt * dt;

        Tangent xi = tangent;

        // Compose: dX_new = dX ⊕ exp(xi)  i.e. right-plus: dX.plus(xi)
        Jacob J_dX;   // ∂(dX ⊕ exp(xi)) / ∂dX  == Adj(exp(xi))^-1
        Jacob J_xi;   // ∂(dX ⊕ exp(xi)) / ∂xi  == Jr
        dX.plus(xi, J_dX, J_xi);

        // --- compute ∂xi/∂b_g and ∂xi/∂b_a (10x3 each) ---
        // measurement noise / bias enters xi linearly:
        // xi.blocks:
        //  rho = cur_dv*dt + 0.5 * (acc_biased - ba) * dt^2   => ∂rho/∂ba = -0.5*dt^2 * I, ∂rho/∂bg = 0
        //  nu  = (acc_biased - ba) * dt                       => ∂nu/∂ba  = -dt * I,      ∂nu/∂bg = 0
        //  theta = (omega_biased - bg) * dt                   => ∂theta/∂bg = -dt * I,   ∂theta/∂ba = 0
        //  s = dt                                             => zeros
        Eigen::Matrix<Scalar,DoF,3> dxi_dbg = Eigen::Matrix<Scalar,DoF,3>::Zero();
        Eigen::Matrix<Scalar,DoF,3> dxi_dba = Eigen::Matrix<Scalar,DoF,3>::Zero();

        // ∂theta/∂bg = -dt * I (theta at indices 6..8)
        dxi_dbg.template block<3,3>(6,0) = -Mat3::Identity() * dt;

        // ∂nu/∂ba = -dt * I (nu at indices 3..5)
        dxi_dba.template block<3,3>(3,0) = -Mat3::Identity() * dt;

        // ∂rho/∂ba = -0.5 * dt^2 * I (rho at indices 0..2)
        dxi_dba.template block<3,3>(0,0) = -Scalar(0.5) * Mat3::Identity() * dt * dt;

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
        // xi = [rho; nu; theta; s]
        // rho: ∂rho/∂n_g = 0, ∂rho/∂n_a = 0.5*dt^2 * I
        G.template block<3,3>(0,3) = Scalar(0.5) * Mat3::Identity() * dt * dt;
        // nu: ∂nu/∂n_a = dt * I
        G.template block<3,3>(3,3) = Mat3::Identity() * dt;
        // theta: ∂theta/∂n_g = dt * I
        G.template block<3,3>(6,0) = Mat3::Identity() * dt;
        // s: zeros

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
    virtual void integrate(const IMUmeas& imu_meas)
    {
        integrate(imu_meas.gyro, imu_meas.accel, 
                  imu_meas.bias.gyro, imu_meas.bias.accel, 
                  imu_meas.dt
                );
    }

    // Apply bias correction: given small bias deltas (delta_bg, delta_ba)
    virtual void correctDelta(const Vec3& delta_bg, const Vec3& delta_ba)
    {
        // correction in tangent: corr = J_bg * delta_bg + J_ba * delta_ba (DoFx1)
        VecTangent corr = VecTangent::Zero();
        corr.template head<3>()  = J_bg.template block<3,3>(0,0) * delta_bg + J_ba.template block<3,3>(0,0) * delta_ba;
        corr.template segment<3>(3) = J_bg.template block<3,3>(3,0) * delta_bg + J_ba.template block<3,3>(3,0) * delta_ba;
        corr.template segment<3>(6) = J_bg.template block<3,3>(6,0) * delta_bg + J_ba.template block<3,3>(6,0) * delta_ba;
        corr(9) = J_bg(9,0)*delta_bg(0) + J_bg(9,1)*delta_bg(1) + J_bg(9,2)*delta_bg(2)
                + J_ba(9,0)*delta_ba(0) + J_ba(9,1)*delta_ba(1) + J_ba(9,2)*delta_ba(2);

        Tangent corrT = corr;
        dX.plus(corrT); // apply correction on right: dX_corr = dX ⊕ Exp(corr)
    }

    // Apply bias correction: given small bias deltas (delta_bg, delta_ba) return corrected dX
    virtual Group getAndCorrectDelta(const Vec3& delta_bg, const Vec3& delta_ba) {
        correctDelta(delta_bg, delta_ba);
        return dX;
    }

    // Group accessor
    Group getState() { return dX; }

    // Time accessor
    Scalar getTotalTime() { return sum_dt; }

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

    // IMU continuous noise spectral densities (per second)
    Mat3 cov_gyro; // (rad/s)^2 / Hz
    Mat3 cov_acc;  // (m/s^2)^2 / Hz

    Scalar sum_dt = Scalar(0);

};

} // namespace lie_odyssey

#endif