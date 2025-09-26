#ifndef __LIEODYSSEY_EKF_INS_HPP__
#define __LIEODYSSEY_EKF_INS_HPP__

#include <Eigen/Dense>
#include "lie_odyssey/core/preintegrator.hpp"

namespace lie_odyssey {

// -------------------- Right iESEKF for Lie Groups --------------------
template <typename Group>
class iESEKF {
public:
    using Scalar  = typename Group::Impl::Native::Scalar;
    using Vec3    = Eigen::Matrix<Scalar,3,1>;
    using Tangent = typename Group::Tangent;
    static constexpr int DoF = Group::Impl::DoF;

    using MatDoF = Eigen::Matrix<Scalar,DoF,DoF>;
    using Mat6   = Eigen::Matrix<Scalar,6,6>;

    // State: Lie group element + biases
    Group X;           // Pose (rotation + translation or full group)
    Vec3 bg;           // Gyro bias
    Vec3 ba;           // Accel bias

    // Covariance on error state (DoF + 6 for biases)
    Eigen::Matrix<Scalar,DoF+6,DoF+6> P;

    // Noise matrices
    Eigen::Matrix<Scalar,3,3> cov_gyro;
    Eigen::Matrix<Scalar,3,3> cov_acc;

    // Preintegrator for IMU propagation
    Preintegrator<Group> preint;

    Scalar tol = Scalar(1e-6);   // convergence tolerance
    int max_iters = 5;           // maximum iterations

    iESEKF()
        : X(), bg(Vec3::Zero()), ba(Vec3::Zero()),
          P(Eigen::Matrix<Scalar,DoF+6,DoF+6>::Zero()),
          cov_gyro(Eigen::Matrix<Scalar,3,3>::Identity()*1e-5),
          cov_acc(Eigen::Matrix<Scalar,3,3>::Identity()*1e-3),
          preint(cov_acc, cov_gyro)
    {}

    // measurement function h(X)
    template <typename MeasurementVec>
    MeasurementVec h(const Group& state) {
        // User-defined measurement function
        MeasurementVec y;
        // fill y using state
        return y;
    }

    // -------------------- Prediction --------------------
    void predict(const Vec3& omega_meas, 
                 const Vec3& acc_meas, 
                 Scalar dt) 
    {
        // Integrate IMU sample with current bias
        preint.integrate(acc_meas, omega_meas, ba, bg, dt);
        Group dX = preint.getState();

        // Right composition: X_next = X ⊕ dX
        X.plus(dX);

        // Assemble augmented error-state transition F (DoF+6)
        Eigen::Matrix<Scalar,DoF,DoF+6> F_aug = Eigen::Matrix<Scalar,DoF,DoF+6>::Zero();
        F_aug.template block<DoF,DoF>(0,0) = Eigen::Matrix<Scalar,DoF,DoF>::Identity(); // J_dX placeholder
        F_aug.template block<DoF,3>(0,DoF) = preint.J_bg;   // ∂Δ/∂bg
        F_aug.template block<DoF,3>(0,DoF+3) = preint.J_ba; // ∂Δ/∂ba

        // Discrete process noise mapped to error-state
        Eigen::Matrix<Scalar,DoF+6,DoF+6> Q_aug = Eigen::Matrix<Scalar,DoF+6,DoF+6>::Zero();
        Q_aug.template block<DoF,DoF>(0,0) = preint.cov; // preintegrated noise

        // Covariance propagation
        P = F_aug * P * F_aug.transpose() + Q_aug;
    }

    // -------------------- Right-Right Measurement Update --------------------
    // y: measurement residual
    // H: Jacobian of measurement w.r.t tangent
    // R: measurement noise
    template <typename MeasurementVec, typename HMat>
    void update(const MeasurementVec& y_meas, 
                const HMat& H, 
                const Eigen::Matrix<Scalar,MeasurementVec::RowsAtCompileTime,MeasurementVec::RowsAtCompileTime>& R) 
    {
        // 1. Predict measurement using current state
        MeasurementVec y_pred = h(X, bg, ba);  // h: measurement model functor

        // 2. Compute residual (innovation)
        MeasurementVec r = y_meas - y_pred;

        // 3. Innovation covariance
        Eigen::Matrix<Scalar, MeasurementVec::RowsAtCompileTime,
                            MeasurementVec::RowsAtCompileTime> S = H * P * H.transpose() + R;

        // 4. Kalman gain
        Eigen::Matrix<Scalar, DoF + 6, MeasurementVec::RowsAtCompileTime> K = P * H.transpose() * S.inverse();

        // 5. Compute error-state correction
        Eigen::Matrix<Scalar, DoF + 6, 1> dx = K * r;

        // 6. Apply correction on Lie group
        Tangent dX_corr = dx.template head<DoF>();
        X.plus(dX_corr);  // Right-plus

        // 7. Correct biases
        bg += dx.template segment<3>(DoF);
        ba += dx.template segment<3>(DoF + 3);

        // 8. Covariance update (Joseph form)
        P = (Eigen::Matrix<Scalar, DoF + 6, DoF + 6>::Identity() - K * H) * P *
            (Eigen::Matrix<Scalar, DoF + 6, DoF + 6>::Identity() - K * H).transpose() + K * R * K.transpose();
    }

    // Iterative update using measurement y and noise R
    template <typename MeasurementVec, typename HMat>
    void updateIterative(const MeasurementVec& y,
                         const Eigen::Matrix<Scalar,MeasurementVec::RowsAtCompileTime,MeasurementVec::RowsAtCompileTime>& R,
                         std::function<HMat(const Group&, const Tangent&)> Hfun) 
    {
        Eigen::Matrix<Scalar, DoF + 6, 1> delta_x = Eigen::Matrix<Scalar, DoF + 6, 1>::Zero();

        for(int iter=0; iter < max_iters; ++iter) {
            // Linearize measurement function around X ⊕ xi
            Tangent xi = delta_x.template head<DoF>();
            HMat H = Hfun(X, xi);
            
            // Compute residual
            MeasurementVec r = y - h(X); // for invariant EKF, may need: y - h(X ⊕ delta_x)
            
            // Kalman gain
            Eigen::Matrix<Scalar,MeasurementVec::RowsAtCompileTime,MeasurementVec::RowsAtCompileTime> S = H * P * H.transpose() + R;
            Eigen::Matrix<Scalar,DoF + 6,MeasurementVec::RowsAtCompileTime> K = P * H.transpose() * S.inverse();
            
            // Update error state
            delta_x += K * r;

            // Check convergence
            if(delta_x.norm() < tol)
                break;
        }

        // Apply final correction
        Tangent dX_corr = delta_x.template head<DoF>();
        X.plus(dX_corr);

        // Update biases
        bg += delta_x.template segment<3>(DoF);
        ba += delta_x.template segment<3>(DoF+3);

        // Covariance update (Joseph form)
        auto H_final = Hfun(X, dX_corr);
        Eigen::Matrix<Scalar,MeasurementVec::RowsAtCompileTime,MeasurementVec::RowsAtCompileTime> S_final = H_final * P * H_final.transpose() + R;
        Eigen::Matrix<Scalar,DoF,MeasurementVec::RowsAtCompileTime> K_final = P * H_final.transpose() * S_final.inverse();
        P = (Eigen::Matrix<Scalar,DoF,DoF>::Identity() - K_final * H_final) * P *
            (Eigen::Matrix<Scalar,DoF,DoF>::Identity() - K_final * H_final).transpose() + K_final * R * K_final.transpose();
    }

    void reset() 
    {
        X.setIdentity();
        bg.setZero();
        ba.setZero();
        P.setZero();
        preint.reset();
    }

    // Utility: update gravity
    void setGravity(const Vec3& g) { preint.updateGravity(g); }
};

} // namespace lie_odyssey


#endif // __LIEODYSSEY_EKF_INS_HPP__