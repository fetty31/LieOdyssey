#ifndef __LIEODYSSEY_CORE_MANIF_PREINTEGRATOR_TRAITS_HPP__
#define __LIEODYSSEY_CORE_MANIF_PREINTEGRATOR_TRAITS_HPP__

#include "lie_odyssey/core/groups.hpp"

namespace lie_odyssey {

#if LIE_BACKEND_MANIF
  template <typename Scalar>
  using SGal3 = LieGroup<Gal3Manif<Scalar>>;
  template <typename Scalar>
  using SO3   = LieGroup<SO3Manif<Scalar>>;
  template <typename Scalar>
  using SE3   = LieGroup<SE3Manif<Scalar>>;
  template <typename Scalar>
  using SE2_3  = LieGroup<SE23Manif<Scalar>>;
#else
  template <typename Scalar>
  using SGal3 = LieGroup<Gal3LiePP<Scalar>>;
  template <typename Scalar>
  using SO3   = LieGroup<SO3LiePP<Scalar>>;
  template <typename Scalar>
  using SE3   = LieGroup<SE3LiePP<Scalar>>;
  template <typename Scalar>
  using SE2_3   = LieGroup<SE23LiePP<Scalar>>;
#endif

// Default traits declaration + definition
template <typename Group>
struct PreintegrationTraits {
    static_assert(sizeof(Group) == 0,
                  "PreintegrationTraits must be specialized for this group.");
};

// Specialization for SGal(3)
template <typename Scalar>
struct PreintegrationTraits<SGal3<Scalar>> {
    using Group   = SGal3<Scalar>;
    using Tangent = typename Group::Tangent;
    using Vec3    = Eigen::Matrix<Scalar,3,1>;
    using Mat3    = Eigen::Matrix<Scalar,3,3>;
    using VecTangent = Eigen::Matrix<Scalar, Group::Impl::DoF, 1>;

    static Tangent get_tangent(const Vec3& acc_unbiased,
                               const Vec3& omega_unbiased,
                               Scalar dt,
                               const Group& dX)
    {
        // Build tangent increment xi for SGal(3) group:
        // xi = [ rho(3); nu(3); theta(3); s(1) ] 
        VecTangent tangent = VecTangent::Zero();

        // nu (acc contribution)
        tangent.template segment<3>(3) = acc_unbiased * dt;

        // theta (rotation)
        tangent.template segment<3>(6) = omega_unbiased * dt;

        // s (time)
        tangent(9) = dt;

        return tangent; // cast
    }

    static void get_dX_dbias(Eigen::Matrix<Scalar,10,3>& dxi_dba,
                             Eigen::Matrix<Scalar,10,3>& dxi_dbg,
                             Scalar dt)
    {
        // --- compute ∂xi/∂b_g and ∂xi/∂b_a for default SGal(3) (10x3 each) ---
        // measurement noise / bias enters xi linearly:
        // xi.blocks:
        //  rho = 0                                            => ∂rho/∂ba = 0, ∂rho/∂bg = 0
        //  nu  = (acc_biased - ba) * dt                       => ∂nu/∂ba  = -dt * I,      ∂nu/∂bg = 0
        //  theta = (omega_biased - bg) * dt                   => ∂theta/∂bg = -dt * I,   ∂theta/∂ba = 0
        //  s = dt                                             => zeros

        dxi_dbg.setZero();
        dxi_dba.setZero();

        // ∂theta/∂bg = -dt * I 
        dxi_dbg.template block<3,3>(6,0) = -Mat3::Identity() * dt;

        // ∂nu/∂ba = -dt * I 
        dxi_dba.template block<3,3>(3,0) = -Mat3::Identity() * dt;
    }

    static void get_meas_noise(Eigen::Matrix<Scalar,10,6>& G, Scalar dt)
    {
        // default SGal(3) xi = [rho; nu; theta; s]
        G.setZero();
        
        // nu: ∂nu/∂n_a = dt * I
        G.template block<3,3>(3,3) = Mat3::Identity() * dt;

        // theta: ∂theta/∂n_g = dt * I
        G.template block<3,3>(6,0) = Mat3::Identity() * dt;

        // rho: ∂rho/∂n_g = 0, ∂rho/∂n_a = 0
        // s: zeros
    }

    static Tangent get_correction(const Eigen::Matrix<Scalar,10,3>& J_ba,
                                  const Eigen::Matrix<Scalar,10,3>& J_bg,
                                  const Vec3& delta_ba,
                                  const Vec3& delta_bg)
    {
        VecTangent corr = VecTangent::Zero();

        corr = J_bg * delta_bg + J_ba * delta_ba;
        corr(9) = Scalar(0); // ensure stays zero (tau should never depend on biases).

        return corr; // cast
    }
};


// Specialization for SO(3)
template <typename Scalar>
struct PreintegrationTraits<SO3<Scalar>> {
    using Group      = SO3<Scalar>;
    using Tangent    = typename Group::Tangent;   // Eigen::Matrix<Scalar,3,1>
    using Jacob      = typename Group::Jacobian;  // Eigen::Matrix<Scalar,3,3>
    using Mat3       = Eigen::Matrix<Scalar,3,3>;
    using Vec3       = Eigen::Matrix<Scalar,3,1>;
    using MatNoise   = Eigen::Matrix<Scalar,3,6>; // maps accel noise also 
    using VecTangent = Eigen::Matrix<Scalar,3,1>;

    // Build tangent increment xi = ω_unbiased * dt
    static Tangent get_tangent(const Vec3& /*acc_unbiased*/,
                               const Vec3& omega_unbiased,
                               Scalar dt,
                               const Group& /*dX*/)
    {
        return omega_unbiased * dt;
    }

    // ∂xi/∂bias
    static void get_dX_dbias(Eigen::Matrix<Scalar,3,3>& dxi_dba,
                             Eigen::Matrix<Scalar,3,3>& dxi_dbg,
                             Scalar dt)
    {
        dxi_dba.setZero();                   // accelerometer irrelevant
        dxi_dbg = -Mat3::Identity() * dt;    // θ = (ω - b_g)*dt
    }

    // measurement noise mapping
    static void get_meas_noise(MatNoise& G, Scalar dt)
    {
        G.setZero();
        // rotation affected by gyro noise
        G.template block<3,3>(3,0) = Mat3::Identity() * dt; // ∂θ/∂n_g = dt * I
    }

    // correction from small bias deltas
    static Tangent get_correction(const Eigen::Matrix<Scalar,3,3>& J_ba,
                                  const Eigen::Matrix<Scalar,3,3>& J_bg,
                                  const Vec3& delta_ba,
                                  const Vec3& delta_bg)
    {
        return J_bg * delta_bg + J_ba * delta_ba;
    }
};

// Specialization for SE3
template <typename Scalar>
struct PreintegrationTraits<SE3<Scalar>> {
    using Group      = SE3<Scalar>;
    using Tangent    = typename Group::Tangent;  // 6x1
    using Vec3       = Eigen::Matrix<Scalar,3,1>;
    using Mat3       = Eigen::Matrix<Scalar,3,3>;
    using VecTangent = Eigen::Matrix<Scalar,6,1>;

    static Tangent get_tangent(const Vec3& acc_unbiased,
                               const Vec3& omega_unbiased,
                               Scalar dt,
                               const Group& dX)
    {
        VecTangent tangent = VecTangent::Zero();

        // translation increment: integrate velocity
        tangent.template segment<3>(0) = Scalar(0.5) * acc_unbiased * dt * dt;

        // rotation increment
        tangent.template segment<3>(3) = omega_unbiased * dt;

        return tangent;
    }

    static void get_dX_dbias(Eigen::Matrix<Scalar,6,3>& dxi_dba,
                             Eigen::Matrix<Scalar,6,3>& dxi_dbg,
                             Scalar dt)
    {
        dxi_dba.setZero();
        dxi_dbg.setZero();

        // rotation depends on gyro bias
        dxi_dbg.template block<3,3>(3,0) = -Mat3::Identity() * dt;

        // translation depends on accel bias
        dxi_dba.template block<3,3>(0,0) = -Scalar(0.5) * Mat3::Identity() * dt * dt;
    }

    static void get_meas_noise(Eigen::Matrix<Scalar,6,6>& G, Scalar dt)
    {
        G.setZero();
        // translation affected by accel noise
        G.template block<3,3>(0,3) = Scalar(0.5) * Mat3::Identity() * dt * dt;
        // rotation affected by gyro noise
        G.template block<3,3>(3,0) = Mat3::Identity() * dt;
    }

    static Tangent get_correction(const Eigen::Matrix<Scalar,6,3>& J_ba,
                                  const Eigen::Matrix<Scalar,6,3>& J_bg,
                                  const Vec3& delta_ba,
                                  const Vec3& delta_bg)
    {
        return J_bg * delta_bg + J_ba * delta_ba;
    }
};

// Specialization for SE23
template <typename Scalar>
struct PreintegrationTraits<SE2_3<Scalar>> {
    using Group      = SE2_3<Scalar>;
    using Tangent    = typename Group::Tangent;       // 9x1
    using Vec3       = Eigen::Matrix<Scalar,3,1>;
    using Mat3       = Eigen::Matrix<Scalar,3,3>;
    using VecTangent = Eigen::Matrix<Scalar,9,1>;

    static Tangent get_tangent(const Vec3& acc_unbiased,
                               const Vec3& omega_unbiased,
                               Scalar dt,
                               const Group& dX)
    {
        VecTangent tangent = VecTangent::Zero();

        // translation increment 
        tangent.template segment<3>(0) = Scalar(0.5) * acc_unbiased * dt * dt;

        // velocity increment
        tangent.template segment<3>(3) = acc_unbiased * dt;

        // rotation increment
        tangent.template segment<3>(6) = omega_unbiased * dt;

        return tangent;
    }

    static void get_dX_dbias(Eigen::Matrix<Scalar,9,3>& dxi_dba,
                             Eigen::Matrix<Scalar,9,3>& dxi_dbg,
                             Scalar dt)
    {
        dxi_dba.setZero();
        dxi_dbg.setZero();

        // rotation bias affects rotation
        dxi_dbg.template block<3,3>(6,0) = -Mat3::Identity() * dt;

        // accel bias affects velocity and translation
        dxi_dba.template block<3,3>(3,0) = -Mat3::Identity() * dt;
        dxi_dba.template block<3,3>(0,0) = -Scalar(0.5) * Mat3::Identity() * dt * dt;
    }

    static void get_meas_noise(Eigen::Matrix<Scalar,9,6>& G, Scalar dt)
    {
        G.setZero();
        // translation rho: 0.5*dt^2 * I (acc)
        G.template block<3,3>(0,3) = Scalar(0.5) * Mat3::Identity() * dt * dt;

        // velocity nu: dt * I (acc)
        G.template block<3,3>(3,3) = Mat3::Identity() * dt;

        // rotation theta: dt * I (gyro)
        G.template block<3,3>(6,0) = Mat3::Identity() * dt;
    }

    static Tangent get_correction(const Eigen::Matrix<Scalar,9,3>& J_ba,
                                  const Eigen::Matrix<Scalar,9,3>& J_bg,
                                  const Vec3& delta_ba,
                                  const Vec3& delta_bg)
    {
        return J_bg * delta_bg + J_ba * delta_ba;
    }
};


} // namespace lie_odyssey

#endif // __LIEODYSSEY_CORE_PREINTEGRATOR_HPP__