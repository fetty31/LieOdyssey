#ifndef __LIEODYSSEY_IESEKF_INS_HPP__
#define __LIEODYSSEY_IESEKF_INS_HPP__

#include "lie_odyssey/ekf/base_filter.hpp"

namespace lie_odyssey {

// -------------------- Right iESEKF for Lie Groups --------------------
template <typename Group>
class iESEKF : public BaseFilter<Group> {
public:

    using Base = BaseFilter<Group>;

    using Scalar  = typename BaseFilter<Group>::Scalar;
    using Vec3    = typename BaseFilter<Group>::Vec3;
    using Tangent = typename BaseFilter<Group>::Tangent;
    static constexpr int DoF = Group::Impl::DoF;

    using MatDoF  = typename BaseFilter<Group>::MatDoF;
    using MatDoFext = typename BaseFilter<Group>::MatDoFext;
    using Mat3 = typename BaseFilter<Group>::Mat3;

    Scalar tol = Scalar(1e-6);   // convergence tolerance
    int max_iters = 5;           // maximum iterations

    iESEKF(const Vec3& ba = Vec3::Zero(), 
        const Vec3& bg = Vec3::Zero(), 
        const MatDoFext& P = MatDoFext::Identity()*Scalar(1e-3),
        const Mat3& cov_acc_init = Mat3::Identity()*Scalar(1e-5),
        const Mat3& cov_gyro_init = Mat3::Identity()*Scalar(1e-3))
        : Base(ba, bg, P, cov_acc_init, cov_gyro_init)
    { }

    // -------------------- Measurement Update --------------------
    // Iterative update 
    template <typename Measurement, typename HMat>
    void updateIterative(const Measurement& y,
                        const Eigen::Matrix<Scalar,
                                            Measurement::DoF, Measurement::DoF>& R,
                        std::function<Measurement(const Group&)> h_fun,
                        std::function<HMat(const Group&)> H_fun)
    {
        Eigen::Matrix<Scalar, DoF + 6, 1> delta_x = Eigen::Matrix<Scalar, DoF + 6, 1>::Zero();

        for(int iter=0; iter < max_iters; ++iter) {
            // Current linearization point
            Tangent xi = delta_x.template head<DoF>();
            Group X_lin = this->X_;   // reference frame
            X_lin.plus(xi);           // new linearization point X ⊕ xi

            // Linearize measurement
            HMat H = H_fun(X_lin);

            // Residual at this point
            Measurement y_pred = h_fun(X_lin);  // h: measurement model functor
            auto r = y.minus(y_pred);           // y - h(X ⊕ xi)
            
            // Kalman gain
            Eigen::Matrix<Scalar,Measurement::DoF,Measurement::DoF> S = H * this->P_ * H.transpose() + R;
            Eigen::Matrix<Scalar,DoF + 6,Measurement::DoF> K = this->P_ * H.transpose() * S.inverse();
            
            // Update error state
            delta_x += K * r;

            // Check convergence
            if(delta_x.norm() < tol)
                break;
        }

        // Apply final correction
        Tangent dX_corr = delta_x.template head<DoF>();
        this->X_.plus(dX_corr);

        // Update biases
        this->ba_ += delta_x.template segment<3>(DoF);
        this->bg_ += delta_x.template segment<3>(DoF+3);

        // Covariance update (Joseph form)
        auto H_final = H_fun(this->X_);
        Eigen::Matrix<Scalar,Measurement::DoF,Measurement::DoF> S_final = H_final * this->P_ * H_final.transpose() + R;
        Eigen::Matrix<Scalar,DoF+6,Measurement::DoF> K_final = this->P_ * H_final.transpose() * S_final.inverse();
        this->P_ = (Base::MatDoF::Identity() - K_final * H_final) * this->P_ *
            (Base::MatDoF::Identity() - K_final * H_final).transpose() + K_final * R * K_final.transpose();
    }

    // Set convergence tolerance
    void setTolerance(const Scalar& t) { tol = t; }

    // Set maximum iterations
    void setIters(const int& iters) { max_iters = iters; }

};

} // namespace lie_odyssey


#endif // __LIEODYSSEY_IESEKF_INS_HPP__