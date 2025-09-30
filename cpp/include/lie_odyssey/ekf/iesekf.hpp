#ifndef __LIEODYSSEY_IESEKF_INS_HPP__
#define __LIEODYSSEY_IESEKF_INS_HPP__

#include "lie_odyssey/ekf/base_filter.hpp"

namespace lie_odyssey {

// -------------------- Right iESEKF for Lie Groups --------------------
template <typename Group>
class iESEKF : public BaseFilter<Group> {
public:

    using Base = BaseFilter<Group>;

    iESEKF(const Vec3& ba = Vec3::Zero(), 
        const Vec3& bg = Vec3::Zero(), 
        const MatDoFext& P = MatDoFext::Identity()*Scalar(1e-3),
        const Mat3& cov_acc_init = Mat3::Identity()*Scalar(1e-5),
        const Mat3& cov_gyro_init = Mat3::Identity()*Scalar(1e-3))
        : Base(ba, bg, P, cov_acc_init, cov_gyro_init);
    { }

    // -------------------- Measurement Update --------------------

    // Iterative update 
    template <typename MeasurementVec, typename HMat>
    void updateIterative(const MeasurementVec& y,
                         const Eigen::Matrix<Scalar,MeasurementVec::RowsAtCompileTime,MeasurementVec::RowsAtCompileTime>& R,
                         std::function<HMat(const Group&)> h_fun,
                         std::function<HMat(const Group&)> H_fun) 
    {
        Eigen::Matrix<Scalar, DoF + 6, 1> delta_x = Eigen::Matrix<Scalar, DoF + 6, 1>::Zero();

        for(int iter=0; iter < max_iters; ++iter) {
            // Current linearization point
            Tangent xi = delta_x.template head<DoF>();
            Group X_lin = X_;   // reference frame
            X_lin.plus(xi);     // new linearization point X ⊕ xi

            // Linearize measurement
            HMat H = H_fun(X_lin);

            // Residual at this point
            MeasurementVec r = y - h_fun(X_lin); // y - h(X ⊕ xi)
            
            // Kalman gain
            Eigen::Matrix<Scalar,MeasurementVec::RowsAtCompileTime,MeasurementVec::RowsAtCompileTime> S = H * P_ * H.transpose() + R;
            Eigen::Matrix<Scalar,DoF + 6,MeasurementVec::RowsAtCompileTime> K = P_ * H.transpose() * S.inverse();
            
            // Update error state
            delta_x += K * r;

            // Check convergence
            if(delta_x.norm() < tol)
                break;
        }

        // Apply final correction
        Tangent dX_corr = delta_x.template head<DoF>();
        X_.plus(dX_corr);

        // Update biases
        ba_ += delta_x.template segment<3>(DoF);
        bg_ += delta_x.template segment<3>(DoF+3);

        // Covariance update (Joseph form)
        auto H_final = H_fun(X_);
        Eigen::Matrix<Scalar,MeasurementVec::RowsAtCompileTime,MeasurementVec::RowsAtCompileTime> S_final = H_final * P_ * H_final.transpose() + R;
        Eigen::Matrix<Scalar,DoF+6,MeasurementVec::RowsAtCompileTime> K_final = P_ * H_final.transpose() * S_final.inverse();
        P_ = (Base::MatDoF::Identity() - K_final * H_final) * P_ *
            (Base::MatDoF::Identity() - K_final * H_final).transpose() + K_final * R * K_final.transpose();
    }

};

} // namespace lie_odyssey


#endif // __LIEODYSSEY_IESEKF_INS_HPP__