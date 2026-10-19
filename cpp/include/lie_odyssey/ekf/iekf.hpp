#ifndef __LIEODYSSEY_RIEKF_FILTER_HPP__
#define __LIEODYSSEY_RIEKF_FILTER_HPP__

#include <Eigen/Dense>
#include <mutex>
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
    //      Note that for this filter to be truly invariant, the current state
    //      given as arg to the process function (TangentFunction) should not be used.
    //      It is left as arg for imperfect-invariant filtering options. 
    using TangentFunction = std::function<Tangent(const Group&, const IMUmeas<Scalar>&)>;
    using JacobianXFun  = std::function<Jacobian(const Group&, const IMUmeas<Scalar>&)>;
    using JacobianWFun  = std::function<MappingMatrix(const Group&, const IMUmeas<Scalar>&)>;
    using DegeneracyCallback = std::function<void(const Group&, Tangent&, const MatDoF&)>;

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
            f_ = [](const Group&, const IMUmeas<Scalar>&) { return VecTangent::Zero(); }; // cast
        }
        if (!f_dx_) {
            f_dx_ = [](const Group&, const IMUmeas<Scalar>&) { return Jacobian::Identity(); };
        }
        if (!f_dw_) {
            f_dw_ = [](const Group&, const IMUmeas<Scalar>&) { return MappingMatrix::Zero(); };
        }

        // Safe callback, do not handle degeneracy
        if (!degeneracy_callback_) {
            degeneracy_callback_ = [](const Group&, Tangent&, const MatDoF&) { return; };
        }
    }

    // -------------------- Prediction --------------------
    virtual void predict(const IMUmeas<Scalar>& imu) 
    {
        std::lock_guard<std::mutex> lock(mutex_); 

        const auto X_k = X_;

        // 1. Predict State: \hat{X}_{k+1} = \hat{X}_k \exp(f(u) * dt)
        // In RI-EKF, we usually integrate the group element directly
        Tangent u_dt = f_(X_k, imu) * Scalar(imu.dt);
        X_.plus(u_dt); 

        // 2. Predict Covariance:
        // For RI-EKF, Fx is the "A" matrix of the linearized error dynamics: \dot{\xi} = A\xi + Ad_X W
        // The discrete-time transition matrix \Phi \= I + Fx*dt (first-order approx)
        Jacobian Fx = f_dx_(X_k, imu); 
        MappingMatrix Fw = f_dw_(X_k, imu);
        Fw = X_k.Adjoint() * Fw; // map body-frame noise to tangent space

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
                std::function<Measurement(const Group&, const Measurement& y)> h_fun,
                std::function<HMat(const Group&)> H_fun)
    {
        std::lock_guard<std::mutex> lock(mutex_); 

        // RI-EKF Update Step
        // 1. Calculate Jacobian H at current estimate
        HMat H = H_fun(X_);

        // 2. Residual (Innovation)
        // For Invariant filters, h_fun should return: \hat{X} * y - b
        Measurement r = h_fun(X_, y);

        // 3. Kalman Gain
        auto S = H * P_ * H.transpose() + R;
        Eigen::Matrix<Scalar, DoF, Eigen::Dynamic> K = (P_ * H.transpose() * S.inverse()).eval();

        // 4. Update State (Right-Invariant: X = \exp(K*r) * X)
        Tangent delta = K * r;
        degeneracy_callback_(X_, delta, H.transpose() * R_inv * H);
        
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
                std::function<void(const Group&, Measurement&, HMat&)> H_fun)
    {
        std::lock_guard<std::mutex> lock(mutex_); 

        Measurement r;
        HMat H;
        H_fun(X_, r, H);

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
        std::lock_guard<std::mutex> lock(mutex_); 

        X_.setIdentity(); 
        P_ = MatDoF::Identity() * Scalar(1e-3); 
    }

    // Access filter covariance
    MatDoF getCovariance() const 
    {
        std::lock_guard<std::mutex> lock(mutex_);  
        return P_; 
    }

    // Access process noise
    NoiseMatrix getProcessNoise() const 
    {
        std::lock_guard<std::mutex> lock(mutex_);  
        return Q_; 
    }

    // Access state
    Group getState() const 
    {
        std::lock_guard<std::mutex> lock(mutex_);  
        return X_; 
    }

    // Set filter covariance
    void setCovariance(const MatDoF& P) 
    {
        std::lock_guard<std::mutex> lock(mutex_);  
        P_ = P; 
    }

    // Set propagation noise
    void setProcessNoise(const NoiseMatrix& Q) 
    {
        std::lock_guard<std::mutex> lock(mutex_);  
        Q_ = Q; 
    }

    // Set state
    void setState(const Group& X) 
    {
        std::lock_guard<std::mutex> lock(mutex_);  
        X_ = X; 
    }

protected:
    // State: Lie group element (or Bundle)
    Group X_;   

    // Covariance on error state (DoF)
    MatDoF P_;

    // Propagation noise matrix
    NoiseMatrix Q_;

    // User-defined dynamics (+ degeneracy handling)
    TangentFunction f_;     // system dynamics (IMU input mapped to tangent space)
    JacobianXFun f_dx_;     // Jacobian w.r.t. state
    JacobianWFun f_dw_;     // Jacobian w.r.t. noise
    DegeneracyCallback degeneracy_callback_;   // Degeneracy handling callback (optional)

    // Protect against parallel use
    mutable std::mutex mutex_;
};

} // namespace lie_odyssey

#endif