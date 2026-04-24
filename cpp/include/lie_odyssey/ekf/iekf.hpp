#ifndef __LIEODYSSEY_RIEKF_FILTER_HPP__
#define __LIEODYSSEY_RIEKF_FILTER_HPP__

#include <Eigen/Dense>
#include "lie_odyssey/core/groups.hpp"
#include "lie_odyssey/core/imu_data.hpp"

namespace lie_odyssey {

/**
 * @brief Right-Invariant Extended Kalman Filter (RI-EKF)
 * Based on the theory that error is defined as \eta = X * \hat{X}^-1
 */
template <typename Group>
class InvariantEKF {
public:
    using Scalar  = typename Group::Impl::Native::Scalar;
    using Tangent = typename Group::Tangent;
    static constexpr int DoF = Group::Impl::DoF;

    using VecTangent = Eigen::Matrix<Scalar, DoF, 1>;
    
    using Jacobian = typename Group::Jacobian;          
    using NoiseMatrix = Eigen::Matrix<Scalar, 12, 12>;    

    using MatDoF = Eigen::Matrix<Scalar, DoF, DoF>;
    using MappingMatrix = Eigen::Matrix<Scalar, DoF, 12>;

    // User-defined dynamics (Invariant dynamics: \dot{X} = f(X, u))
    using TangentFunction = std::function<Tangent(const InvariantEKF<Group>&, const IMUmeas&)>;
    using JacobianXFun  = std::function<Jacobian(const InvariantEKF<Group>&, const IMUmeas&)>;
    using JacobianWFun  = std::function<MappingMatrix(const InvariantEKF<Group>&, const IMUmeas&)>;
    using DegeneracyCallback = std::function<void(const InvariantEKF<Group>&, Tangent&, const MatDoF&)>;

    InvariantEKF(
           const MatDoF& P = MatDoF::Identity()*Scalar(1e-3),
           const NoiseMatrix& Q = NoiseMatrix::Identity()*Scalar(1e-3),
           TangentFunction f = nullptr,
           JacobianXFun f_dx = nullptr,
           JacobianWFun f_dw = nullptr,
           DegeneracyCallback degen_callback = nullptr)
        : X_(), P_(P), Q_(Q),
        f_(f), f_dx_(f_dx), f_dw_(f_dw), degeneracy_callback_(degen_callback)
    {
        // Provide safe defaults (identity dynamics)
        if (!f_) {
            f_ = [](const InvariantEKF<Group>&, const IMUmeas&) { return VecTangent::Zero(); }; // cast
        }
        if (!f_dx_) {
            f_dx_ = [](const InvariantEKF<Group>&, const IMUmeas&) { return Jacobian::Identity(); };
        }
        if (!f_dw_) {
            f_dw_ = [](const InvariantEKF<Group>&, const IMUmeas&) { return MappingMatrix::Zero(); };
        }

        // Safe callback, do not handle degeneracy
        if (!degeneracy_callback_) {
            degeneracy_callback_ = [](const InvariantEKF<Group>&, Tangent&, const MatDoF&) { return; };
        }
    }

    // -------------------- Prediction --------------------
    virtual void predict(const IMUmeas& imu) 
    {
        // 1. Predict State: \hat{X}_{k+1} = \hat{X}_k \exp(f(u) * dt)
        // In RI-EKF, we usually integrate the group element directly
        Tangent u_dt = f_(*this, imu) * Scalar(imu.dt);
        X_.plus(u_dt); 

        // 2. Predict Covariance:
        // For RI-EKF, Fx is the "A" matrix of the linearized error dynamics: \dot{\xi} = A\xi + Ad_X W
        // The discrete-time transition matrix \Phi \= I + Fx*dt (first-order approx)
        Jacobian Fx = f_dx_(*this, imu); 
        MappingMatrix Fw = f_dw_(*this, imu);
        Fw = X_.Adjoint() * Fw; // map body-frame noise to tangent space

        // Discrete time transition \Phi = expm(Fx * dt)
        MatDoF Phi = (MatDoF::Identity() + Fx * Scalar(imu.dt)); 

        P_ = Phi * P_ * Phi.transpose() + (Fw * Q_ * Fw.transpose()) * Scalar(imu.dt); 
    }

    // -------------------- Measurement Update --------------------
    // In RI-EKF, the innovation is typically: r = \hat{X} * y - b 
    // where b is the constant observation in the fixed frame.
    template <typename Measurement, typename HMat>
    void update(const Measurement& y,
                const Eigen::Matrix<Scalar, -1, -1>& R,
                const Eigen::Matrix<Scalar, -1, -1>& R_inv,
                std::function<Measurement(const InvariantEKF<Group>&, const Group&, const Measurement& y)> h_fun,
                std::function<HMat(const InvariantEKF<Group>&, const Group&)> H_fun)
    {
        // RI-EKF Update Step
        // 1. Calculate Jacobian H at current estimate
        HMat H = H_fun(*this, X_);

        // 2. Residual (Innovation)
        // For Invariant filters, h_fun should return: \hat{X} * y - b
        Measurement r = h_fun(*this, X_, y);

        // 3. Kalman Gain
        auto S = H * P_ * H.transpose() + R;
        Eigen::Matrix<Scalar, DoF, Eigen::Dynamic> K = (P_ * H.transpose() * S.inverse()).eval();

        // 4. Update State (Right-Invariant: X = \exp(K*r) * X)
        Tangent delta = K * r;
        degeneracy_callback_(*this, delta, H.transpose() * R_inv * H);
        
        X_.lplus(delta); // Right-Invariant update (left plus)

        // 5. Covariance Update (Joseph Form)
        MatDoF I = MatDoF::Identity();
        MatDoF KH = K * H;
        P_ = (I - KH) * P_ * (I - KH).transpose() + K * R * K.transpose();
        
        // Enforce symmetry
        P_ = 0.5 * (P_ + P_.transpose());
    }

    // Parity with the iterative interface
    template <typename Measurement, typename HMat>
    void update(Scalar R,
                std::function<void(const InvariantEKF<Group>&, const Group&, Measurement&, HMat&)> H_fun)
    {
        Measurement r;
        HMat H;
        H_fun(*this, X_, r, H);

        MatDoF S = H * P_ * H.transpose() + MatDoF::Identity() * R;
        Eigen::Matrix<Scalar, DoF, Eigen::Dynamic> K = P_ * H.transpose() * S.inverse();

        Tangent delta = K * r;
        X_.lplus(delta); // Right-Invariant update (left plus)

        MatDoF I = MatDoF::Identity();
        P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R * K.transpose();
        P_ = 0.5 * (P_ + P_.transpose());
    }

    // Boilerplate Getters/Setters
    void reset() 
    { 
        X_.setIdentity(); 
        P_ = MatDoF::Identity() * Scalar(1e-3); 
    }

    // Access filter covariance
    MatDoF getCovariance() const { return P_; }

    // Access state
    Group getState() const { return X_; }

    // Set filter covariance
    void setCovariance(const MatDoF& P) { P_ = P; }

    // Set state
    void setState(const Group& X) { X_ = X; }

protected:
    Group X_;   
    MatDoF P_;
    NoiseMatrix Q_;

    TangentFunction f_;     
    JacobianXFun f_dx_;     
    JacobianWFun f_dw_;     
    DegeneracyCallback degeneracy_callback_; 
};

} // namespace lie_odyssey

#endif