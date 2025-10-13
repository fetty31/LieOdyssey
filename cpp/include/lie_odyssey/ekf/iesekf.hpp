#ifndef __LIEODYSSEY_IESEKF_FILTER_HPP__
#define __LIEODYSSEY_IESEKF_FILTER_HPP__

#include <Eigen/Dense>
#include "lie_odyssey/core/groups.hpp"
#include "lie_odyssey/core/imu_data.hpp"

namespace lie_odyssey {

// -------------------- Iterative Error State Extended Kalman Filter (iESEKF) on Manifolds --------------------
template <typename Group>
class iESEKF {
public:
    using Scalar  = typename Group::Impl::Native::Scalar;
    using Tangent = typename Group::Tangent;
    static constexpr int DoF = Group::Impl::DoF;

    using VecTangent = Eigen::Matrix<Scalar, DoF, 1>;

    using Jacobian = typename Group::Jacobian;          // same as MatDoF
    using NoiseMatrix = Eigen::Matrix<Scalar,12,12>;    // w = (n_w, n_a, n_{b_w}, n_{b_a})

    using MatDoF = Eigen::Matrix<Scalar,DoF,DoF>;
    using MappingMatrix = Eigen::Matrix<Scalar,DoF,12>;

    // User-defined dynamics
    using TangentFunction = std::function<Tangent(const iESEKF&, const IMUmeas&)>;
    using JacobianXFun  = std::function<Jacobian(const iESEKF&, const IMUmeas&)>;
    using JacobianWFun  = std::function<MappingMatrix(const iESEKF&, const IMUmeas&)>;

    // State: Lie group element (or Bundle)
    Group X_;   

    // Covariance on error state (DoF)
    MatDoF P_;

    // Propagation noise matrix
    NoiseMatrix Q_;

    // Maximum iterations
    int max_iters_;

    // Tolerance
    Scalar tol_;

    iESEKF(
           const MatDoF& P = MatDoF::Identity()*Scalar(1e-3),
           const NoiseMatrix& Q = NoiseMatrix::Identity()*Scalar(1e-3),
           TangentFunction f = nullptr,
           JacobianXFun f_dx = nullptr,
           JacobianWFun f_dw = nullptr)
        : X_(), P_(P), Q_(Q),
        f_(f), f_dx_(f_dx), f_dw_(f_dw), 
        max_iters_(3), tol_(Scalar(1e-9))
    {
        // Provide safe defaults (identity dynamics)
        if (!f_) {
            f_ = [](const iESEKF&, const IMUmeas&) { return VecTangent::Zero(); }; // cast
        }
        if (!f_dx_) {
            f_dx_ = [](const iESEKF&, const IMUmeas&) { return Jacobian::Identity(); };
        }
        if (!f_dw_) {
            f_dw_ = [](const iESEKF&, const IMUmeas&) { return MappingMatrix::Zero(); };
        }
    }

    // -------------------- Prediction --------------------
    virtual void predict(const IMUmeas& imu) 
    {
        // Propagate state using user dynamics
        Jacobian J_dX;   // ∂(dX ⊕ exp(xi)) / ∂dX  == Adj(exp(xi))^-1
        Jacobian J_xi;   // ∂(dX ⊕ exp(xi)) / ∂xi  == Jr
        X_.plus(f_(*this, imu) * imu.dt, J_dX, J_xi);

        // Update covariance
        Jacobian Fx = J_dX + J_xi * f_dx_(*this, imu) * imu.dt; // He-2021, [https://arxiv.org/abs/2102.03804] Eq. (26)
        MappingMatrix Fw = J_xi * f_dw_(*this, imu) * imu.dt;     // He-2021, [https://arxiv.org/abs/2102.03804] Eq. (27)

        P_ = Fx * P_ * Fx.transpose() + Fw * Q_ * Fw.transpose(); 
    }

    // -------------------- Measurement Update --------------------
    // y: measurement
    // R: measurement noise
    // h_fun: measurement function returning residual (y-ypred)
    // H_fun: Jacobian of measurement w.r.t tangent function
    template <typename Measurement, typename HMat>
    void update(const Measurement& y,
                const Eigen::Matrix<Scalar,
                                    Eigen::Dynamic, Eigen::Dynamic>& R,
                std::function<Measurement(const iESEKF&, const Group&, const Measurement& y)> h_fun,
                std::function<HMat(const iESEKF&, const Group&)> H_fun)
    {

        Group X_now = X_;   // predicted state (reference frame)
        MatDoF P_now = P_;  // predicted covariance
        
        auto R_inv = R.inverse();

        MatDoF KH = MatDoF::Zero();

        for(int iter=0; iter < max_iters_; ++iter) {

            // Current error state
            Jacobian J;
            Tangent dx = X_now.minus(X_, J);  // Xu-2021, [https://arxiv.org/abs/2107.06829] Eq. (11)
            X_now.plus(dx);                   // new linearization point X ⊕ dx

            // Linearize measurement
            HMat H = H_fun(*this, X_now);       // (Eigen::Dynamic x DoF) = (N measurements x DoF)

            // Residual at this point
            Measurement r = h_fun(*this, X_now, y);   // y - h(X ⊕ dx)

            // Update covariance
            Jacobian J_inv = J.inverse();
            P_now = J_inv * P_ * J_inv.transpose();

            // Kalman gain (K = (HT R^−1 H + P^−1)^−1 HT R^−1)
            MatDoF HRH = H.transpose() * R_inv * H; // (HT R^−1 H)
            MatDoF aux = P_now.inverse();           // (P^−1)
            aux += HRH;                             
            aux = aux.inverse();                    // (HT R^−1 H + P^−1)^−1

            Eigen::Matrix<Scalar,DoF,Eigen::Dynamic> K = aux * H.transpose() * R_inv;

            // Update error state
            KH = K*H;

            dx = K*r.vector() + (KH - MatDoF::Identity()) * J_inv * dx; 
            X_now.plus(dx);

            // Check convergence
            if(dx.coeffs().norm() < tol_)
                break;
        }

        // Apply final correction
        X_ = X_now;

        // Covariance update
        P_ = (MatDoF::Identity() - KH) * P_;
    }

    // -------------------- Measurement Update --------------------
    // R: measurement noise
    // H_fun: measurement function -> fills residual z and measurement jacobian H
    template <typename Measurement, typename HMat>
    void update(const Eigen::Matrix<Scalar,
                                    Eigen::Dynamic, Eigen::Dynamic>& R,
                std::function<void(const iESEKF&, const Group&, Measurement&, HMat&)> H_fun)
    {

        Group X_now = X_;   // predicted state (reference frame)
        MatDoF P_now = P_;  // predicted covariance
        
        auto R_inv = R.inverse();

        MatDoF KH = MatDoF::Zero();

        for(int iter=0; iter < max_iters_; ++iter) {

            // Current error state
            Jacobian J;
            Tangent dx = X_now.minus(X_, J);  // Xu-2021, [https://arxiv.org/abs/2107.06829] Eq. (11)
            X_now.plus(dx);                   // new linearization point X ⊕ dx

            // Get residual and linearized measurement model
            Measurement r;
            HMat H;
            H_fun(*this, X_now, r, H);       // H == (Eigen::Dynamic x DoF) = (N measurements x DoF)
                                             // r == (Eigen::Dynamic x 1) = (N measurements x 1)

            // Update covariance
            Jacobian J_inv = J.inverse();
            P_now = J_inv * P_ * J_inv.transpose();

            // Kalman gain (K = (HT R^−1 H + P^−1)^−1 HT R^−1)
            MatDoF HRH = H.transpose() * R_inv * H; // (HT R^−1 H)
            MatDoF aux = P_now.inverse();           // (P^−1)
            aux += HRH;                             
            aux = aux.inverse();                    // (HT R^−1 H + P^−1)^−1

            Eigen::Matrix<Scalar,DoF,Eigen::Dynamic> K = aux * H.transpose() * R_inv;

            // Update error state
            KH = K*H;

            dx = K*r.vector() + (KH - MatDoF::Identity()) * J_inv * dx; 
            X_now.plus(dx);

            // Check convergence
            if(dx.coeffs().norm() < tol_)
                break;
        }

        // Apply final correction
        X_ = X_now;

        // Covariance update
        P_ = (MatDoF::Identity() - KH) * P_;
    }

    // -------------------- Measurement Update --------------------
    // R: measurement noise
    // H_fun: measurement function -> fills residual z and measurement jacobian H
    template <typename Measurement, typename HMat>
    void update(Scalar R,
                std::function<void(const iESEKF&, const Group&, Measurement&, HMat&)> H_fun)
    {

        Group X_now = X_;   // predicted state (reference frame)
        MatDoF P_now = P_;  // predicted covariance
        
        MatDoF KH = MatDoF::Zero();

        for(int iter=0; iter < max_iters_; ++iter) {

            // Current error state
            Jacobian J;
            Tangent dx = X_now.minus(X_, J);  // Xu-2021, [https://arxiv.org/abs/2107.06829] Eq. (11)
            X_now.plus(dx);                   // new linearization point X ⊕ dx

            // Get residual and linearized measurement model
            Measurement r;
            HMat H;
            H_fun(*this, X_now, r, H);       // H == (Eigen::Dynamic x DoF) = (N measurements x DoF)
                                             // r == (Eigen::Dynamic x 1) = (N measurements x 1)

            // Update covariance
            Jacobian J_inv = J.inverse();
            P_now = J_inv * P_ * J_inv.transpose();

            // Kalman gain (K = (HT R^−1 H + P^−1)^−1 HT R^−1)
            MatDoF HRH = H.transpose() * H / R; // (HT R^−1 H)
            MatDoF aux = P_now.inverse();           // (P^−1)
            aux += HRH;                             
            aux = aux.inverse();                    // (HT R^−1 H + P^−1)^−1

            Eigen::Matrix<Scalar,DoF,Eigen::Dynamic> K = aux * H.transpose() / R;

            // Update error state
            KH = K*H;

            dx = K*r.vector() + (KH - MatDoF::Identity()) * J_inv * dx; 
            X_now.plus(dx);

            // Check convergence
            if(dx.coeffs().norm() < tol_)
                break;
        }

        // Apply final correction
        X_ = X_now;

        // Covariance update
        P_ = (MatDoF::Identity() - KH) * P_;
    }

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

    // Set max iters
    void setMaxIters(int it) { max_iters_ = it; }

    // Set max iters
    void setTolerance(Scalar tol) { tol_ = tol; }

    protected:
        TangentFunction f_;     // system dynamics (IMU input mapped to tangent space)
        JacobianXFun f_dx_;     // Jacobian w.r.t. state
        JacobianWFun f_dw_;     // Jacobian w.r.t. noise

};

} // namespace lie_odyssey


#endif // __LIEODYSSEY_IESEKF_FILTER_HPP__