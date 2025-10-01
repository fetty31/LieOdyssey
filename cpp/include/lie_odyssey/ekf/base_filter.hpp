#ifndef __LIEODYSSEY_EKF_BASE_FILTER_HPP__
#define __LIEODYSSEY_EKF_BASE_FILTER_HPP__

#include <Eigen/Dense>
#include "lie_odyssey/core/preintegrator.hpp"

namespace lie_odyssey {

// -------------------- Base KF filter for Lie Groups --------------------
template <typename Group>
class BaseFilter {
public:
    using Scalar  = typename Group::Impl::Native::Scalar;
    using Vec3    = Eigen::Matrix<Scalar,3,1>;
    using Tangent = typename Group::Tangent;
    static constexpr int DoF = Group::Impl::DoF;

    using MatDoF = Eigen::Matrix<Scalar,DoF,DoF>;
    using MatDoFext = Eigen::Matrix<Scalar,DoF+6,DoF+6>;
    using Mat3 = Eigen::Matrix<Scalar,3,3>;

    // State: Lie group element + biases
    Group X_;           // Lie Group
    Vec3 bg_;           // Gyro bias
    Vec3 ba_;           // Accel bias

    // Covariance on error state (DoF + 6 for biases)
    MatDoFext P_;

    // Noise matrices
    Mat3 cov_gyro;
    Mat3 cov_acc;

    // Preintegrator for IMU propagation
    Preintegrator<Group> preint;

    BaseFilter(const Vec3& ba = Vec3::Zero(), 
                const Vec3& bg = Vec3::Zero(), 
                const MatDoFext& P = MatDoFext::Identity()*Scalar(1e-3),
                const Mat3& cov_acc_init = Mat3::Identity()*Scalar(1e-5),
                const Mat3& cov_gyro_init = Mat3::Identity()*Scalar(1e-3))
        : X_(), bg_(bg), ba_(ba),
          P_(P),
          cov_gyro(cov_gyro_init),
          cov_acc(cov_acc_init),
          preint(cov_acc, cov_gyro)
    {}

    virtual void propagate(const IMUmeas& imu_meas)
    {
        imu_meas.bias.gyro = bg_;
        imu_meas.bias.accel = ba_;
        preint.integrate(imu_meas);
    }

    // -------------------- Prediction --------------------
    virtual void predict() 
    {
        // Integrate IMU sample with current bias
        Group dX = preint.getState();

        // Right composition: X_next = X ⊕ dX
        X_.plus(dX);

        // Assemble augmented error-state transition F (DoF+6)
        Eigen::Matrix<Scalar,DoF,DoF+6> F_aug = Eigen::Matrix<Scalar,DoF,DoF+6>::Zero();
        F_aug.template block<DoF,DoF>(0,0) = MatDoF::Identity();  // J_dX placeholder
        F_aug.template block<DoF,3>(0,DoF) = preint.get_J_ba();   // ∂Δ/∂ba
        F_aug.template block<DoF,3>(0,DoF+3) = preint.get_J_bg(); // ∂Δ/∂bg

        // Discrete process noise mapped to error-state
        MatDoFext Q_aug = MatDoFext::Zero();
        Q_aug.template block<DoF,DoF>(0,0) = preint.getCovariance(); // preintegrated noise

        // Covariance propagation
        P_ = F_aug * P_ * F_aug.transpose() + Q_aug;

        // Reset preintegrator after propagation
        preint.reset();
    }

    virtual void predict(const IMUmeas& imu_meas) 
    {
        // Integrate IMU sample with current bias
        this->propagate(imu_meas);
        this->predict();
    }

    // -------------------- Measurement Update --------------------
    // y: measurement residual
    // R: measurement noise
    // h_fun: measurement function
    // H: Jacobian of measurement w.r.t tangent
    template <typename Measurement, typename HMat>
    void update(const Measurement& y,
                const Eigen::Matrix<Scalar,
                                    Measurement::DoF, Measurement::DoF>& R,
                std::function<Measurement(const Group&)> h_fun,
                const HMat& H)
    {
        // 1. Predict measurement using current state
        Measurement y_pred = h_fun(X_); // h: measurement model functor

        // 2. Innovation (residual with Lie-aware minus)
        auto r = y.minus(y_pred);

        // 3. Innovation covariance
        Eigen::Matrix<Scalar, Measurement::DoF,
                            Measurement::DoF> S = H * P_ * H.transpose() + R;

        // 4. Kalman gain
        Eigen::Matrix<Scalar, DoF + 6, Measurement::DoF> K = P_ * H.transpose() * S.inverse();

        // 5. Compute error-state correction
        Eigen::Matrix<Scalar, DoF + 6, 1> dx = K * r;

        // 6. Apply correction on Lie group
        Tangent dX_corr = dx.template head<DoF>();
        X_.plus(dX_corr);  // Right-plus

        // 7. Correct biases
        ba_ += dx.template segment<3>(DoF);
        bg_ += dx.template segment<3>(DoF + 3);

        // 8. Covariance update (Joseph form)
        P_ = (MatDoFext::Identity() - K * H) * P_ *
            (MatDoFext::Identity() - K * H).transpose() + K * R * K.transpose();
    }

    void reset() 
    {
        X_.setIdentity();
        bg_.setZero();
        ba_.setZero();
        P_ = MatDoFext::Identity() * Scalar(1e-3);
        preint.reset();
    }

    // Update gravity
    void updateGravity(const Vec3& g) { preint.updateGravity(g); }

    // Access filter covariance
    MatDoFext getCovariance() const { return P_; }

    // Set filter covariance
    void setCovariance(const MatDoFext& P) { P_ = P; }

    // Access biases
    Vec3 getGyroBias() const { return bg_; }
    Vec3 getAccelBias() const { return ba_; }

    // Set biases
    void setGyroBias(const Vec3& b) { bg_ = b; }
    void setAccelBias(const Vec3& b) { ba_ = b; }

};

} // namespace lie_odyssey


#endif // __LIEODYSSEY_EKF_BASE_FILTER_HPP__