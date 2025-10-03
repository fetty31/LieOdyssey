#ifndef __LIEODYSSEY_IESEKF_FILTER_HPP__
#define __LIEODYSSEY_IESEKF_FILTER_HPP__

#include <Eigen/Dense>
#include "lie_odyssey/core/groups.hpp"
#include "lie_odyssey/core/imu_data.hpp"

namespace lie_odyssey {

// -------------------- Iterative Error State Extended Kalman Filter (iESEKF) on Manifolds --------------------
template <typename Group>
class BaseFilter {
public:
    using Scalar  = typename Group::Impl::Native::Scalar;
    using Tangent = typename Group::Tangent;
    static constexpr int DoF = Group::Impl::DoF;

    using Jacobian = typename Group::Jacobian;          // same as MatDoF
    using NoiseMatrix = Eigen::Matrix<Scalar,12,12>;    // w = (n_w, n_a, n_{b_w}, n_{b_a})

    using MatDoF = Eigen::Matrix<Scalar,DoF,DoF>;

    // User-defined dynamics
    using TangentFunction = std::function<Tangent(const BaseFilter&, const IMUmeas&)>;
    using JacobianXFun  = std::function<Jacobian(const BaseFilter&, const IMUmeas&)>;
    using JacobianWFun  = std::function<NoiseMatrix(const BaseFilter&, const IMUmeas&)>;

    // State: Lie group element (or Bundle)
    Group X_;   

    // Covariance on error state (DoF)
    MatDoF P_;

    // Propagation noise matrix
    NoiseMatrix Q_;

    // Measurement noise matrix
    MatDoF R_;

    BaseFilter(
           const MatDoF& P = MatDoF::Identity()*Scalar(1e-3),
           const NoiseMatrix& Q = NoiseMatrix::Identity()*Scalar(1e-3),
           const MatDoF& R = MatDoF::Identity()*Scalar(1e-3),
           TangentFunction f = nullptr,
           JacobianXFun f_dx = nullptr,
           JacobianWFun f_dw = nullptr)
        : X_(), P_(P), Q_(Q), R_(R),
        f_(f), f_dx_(f_dx), f_dw_(f_dw)
    {
        // Provide safe defaults (identity dynamics)
        if (!f_) {
            f_ = [](const BaseFilter& filter, const IMUmeas& imu) { return Tangent::Zero(); };
        }
        if (!f_dx_) {
            f_dx_ = [](const BaseFilter&, const IMUmeas&) { return Jacobian::Identity(); };
        }
        if (!f_dw_) {
            f_dw_ = [](const BaseFilter&, const IMUmeas&) { return Jacobian::Identity(); };
        }
    }

    // -------------------- Prediction --------------------
    virtual void predict(const IMUmeas& imu) 
    {
        // Propagate state using user dynamics
        Group dX = f_(*this, imu) * imu.dt;
        Jacobian J_dX;   // ∂(dX ⊕ exp(xi)) / ∂dX  == Adj(exp(xi))^-1
        Jacobian J_xi;   // ∂(dX ⊕ exp(xi)) / ∂xi  == Jr
        X_.plus(dX, J_dX, J_xi);

        // Update covariance
        Jacobian Fx = J_dX + J_xi * f_dx_(*this, imu) * imu.dt; // He-2021, [https://arxiv.org/abs/2102.03804] Eq. (26)
        NoiseMatrix Fw = J_xi * f_dw_(*this, imu) * imu.dt;     // He-2021, [https://arxiv.org/abs/2102.03804] Eq. (27)

        P_ = Fx * P_ * Fx.transpose() + Fw * Q_ * Fw.transpose(); 
    }

    // -------------------- Measurement Update --------------------
    // y: measurement residual
    // R: measurement noise
    // h_fun: measurement function returning residual (y-ypred)
    // H_fun: Jacobian of measurement w.r.t tangent function
    template <typename Measurement, typename HMat>
    void update(const Measurement& y,
                const Eigen::Matrix<Scalar,
                                    Measurement::DoF, Measurement::DoF>& R,
                std::function<Measurement(const BaseFilter&, const Group&, const Measurement& y)> h_fun,
                std::function<HMat(const BaseFilter&, const Group&)> H_fun)
    {

        Group X_now = X_;   // predicted state (reference frame)
        MatDoF P_now = P_;  // predicted covariance
        
        MatDoF R_inv = R_.inverse();

        MatDoF KH = MatDoF::Zero();

        for(int iter=0; iter < max_iters; ++iter) {

            // Current error state
            Jacobian J;
            Tangent dx = X_now.minus(X_, J); // Xu-2021, [https://arxiv.org/abs/2107.06829] Eq. (11)
            X_now.plus(dx);  // new linearization point X ⊕ dx

            // Linearize measurement
            HMat H = H_fun(*this, X_now);

            // Residual at this point
            Measurement r = h_fun(*this, X_now, y);   // // y - h(X ⊕ dx)

            // Update covariance
            Jacobian J_inv = J.inverse();
            P_now = J_inv * P_ * J_inv.transpose();

            // Kalman gain
            HMat HRH = H.transpose() * R_inv * H;
            MatDoF aux = P_now.inverse();
            aux.block<Measurement::DoF, Measurement::DoF>(0, 0) += HRH;
            aux = aux.inverse();

            Eigen::Matrix<Scalar,DoF,Measurement::DoF> K = aux.block<DoF, Measurement::DoF>(0, 0) * H.transpose() * R_inv;

            // Update error state
            KH = K*H;

            dx = K*r + (KH - MatDoF::Identity()) * J_inv * dx; 
            X_now = X_now.plus(dx);

            // Check convergence
            if(dx.norm() < tol)
                break;
        }


        // Apply final correction
        X_ = X_now;
        // X_.plus(dx);

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

    // Set filter covariance
    void setCovariance(const MatDoF& P) { P_ = P; }

    protected:
        TangentFunction f_;     // system dynamics (IMU input mapped to tangent space)
        JacobianXFun f_dx_;     // Jacobian w.r.t. state
        JacobianWFun f_dw_;     // Jacobian w.r.t. noise

};

} // namespace lie_odyssey


#endif // __LIEODYSSEY_IESEKF_FILTER_HPP__