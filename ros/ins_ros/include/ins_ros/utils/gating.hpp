#pragma once

#include "ins_ros/utils/chi_square.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>

namespace ins_ros::utils::gating {

/**
 * @brief Outcome of a normalized innovation squared (NIS) computation.
 */
enum class NisStatus
{
    Ok = 0,
    InvalidDimensions,
    NonFinite,
    SingularInnovationCovariance,
    NotPositiveDefinite
};

inline const char* to_string(NisStatus status)
{
    switch (status)
    {
        case NisStatus::Ok:
            return "ok";
        case NisStatus::InvalidDimensions:
            return "dimension mismatch between residual, Jacobian, covariance and noise";
        case NisStatus::NonFinite:
            return "non-finite residual or innovation covariance";
        case NisStatus::SingularInnovationCovariance:
            return "innovation covariance is singular";
        case NisStatus::NotPositiveDefinite:
            return "innovation covariance is not positive definite";
    }

    return "unknown";
}

/**
 * @brief Result of the innovation validation: NIS plus the quantities needed
 *        for logging (residual norm and innovation covariance).
 */
struct NisResult
{
    NisStatus status{NisStatus::InvalidDimensions};
    int dimension{0};               //!< Measurement dimension (degrees of freedom of the test).
    double nis{0.0};                //!< Normalized innovation squared, r^T S^-1 r.
    double innovation_norm{0.0};    //!< Euclidean norm of the residual, ||r||.
    Eigen::MatrixXd innovation_covariance;  //!< S = H P H^T + R.

    bool valid() const { return status == NisStatus::Ok; }
};

/**
 * @brief Accept/reject decision of a chi-square (Mahalanobis) gate.
 */
struct GateDecision
{
    bool accepted{false};
    double nis{0.0};
    double threshold{0.0};  //!< Chi-square threshold for the given dof/confidence.
    int dimension{0};
};

/**
 * @brief Normalized innovation squared of a measurement.
 *
 *   S   = H P H^T + R
 *   NIS = r^T S^{-1} r
 *
 * S^{-1} r is obtained from an LDLT factorization of S instead of forming the
 * explicit inverse, which is both faster and numerically safer.
 *
 * @param residual Residual r = z - h(x) (N x 1).
 * @param H        Measurement Jacobian (N x DoF).
 * @param P        State covariance (DoF x DoF).
 * @param R        Measurement noise covariance (N x N).
 */
template <typename ResidualType, typename HType, typename PType, typename RType>
inline NisResult compute_nis(const ResidualType& residual,
                             const HType& H,
                             const PType& P,
                             const RType& R)
{
    using Scalar = typename HType::Scalar;
    using MatDyn = Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic>;

    NisResult result;
    result.dimension = static_cast<int>(residual.rows());
    result.innovation_norm = static_cast<double>(residual.norm());

    if (residual.rows() == 0 ||
        H.rows() != residual.rows() ||
        H.cols() != P.rows() ||
        H.cols() != P.cols() ||
        R.rows() != residual.rows() ||
        R.cols() != residual.rows())
    {
        result.status = NisStatus::InvalidDimensions;
        return result;
    }

    MatDyn S = H * P * H.transpose();
    S += R;
    S = (Scalar(0.5) * (S + S.transpose())).eval();  // symmetrize (round-off)

    if (!S.allFinite() || !residual.allFinite())
    {
        result.status = NisStatus::NonFinite;
        return result;
    }

    Eigen::LDLT<MatDyn> ldlt(S);

    if (ldlt.info() != Eigen::Success)
    {
        result.status = NisStatus::SingularInnovationCovariance;
        return result;
    }

    if ((ldlt.vectorD().array() <= Scalar(0)).any())
    {
        result.status = NisStatus::NotPositiveDefinite;
        return result;
    }

    const Eigen::Matrix<Scalar, Eigen::Dynamic, 1> x = ldlt.solve(residual);

    if (ldlt.info() != Eigen::Success || !x.allFinite())
    {
        result.status = NisStatus::SingularInnovationCovariance;
        return result;
    }

    const Scalar nis = residual.dot(x);

    if (!std::isfinite(static_cast<double>(nis)) || nis < Scalar(0))
    {
        result.status = NisStatus::NonFinite;
        return result;
    }

    result.status = NisStatus::Ok;
    result.nis = static_cast<double>(nis);
    result.innovation_covariance = S;

    return result;
}

/**
 * @brief Chi-square gate: accept the measurement iff NIS <= chi2(d, confidence).
 *
 * The threshold is computed from the measurement dimension d and the
 * configurable confidence level, it is never hard-coded.
 *
 * A numerically invalid NIS is rejected: an unusable innovation covariance must
 * never reach the update.
 */
inline GateDecision evaluate(const NisResult& nis, double confidence)
{
    GateDecision decision;
    decision.dimension = nis.dimension;
    decision.nis = nis.nis;
    decision.threshold =
        chi_square::quantile(nis.dimension, confidence);
    decision.accepted =
        nis.valid() &&
        std::isfinite(decision.threshold) &&
        (nis.nis <= decision.threshold);

    return decision;
}

/**
 * @brief Compact single-line representation of a matrix for logging.
 */
inline std::string to_string(const Eigen::MatrixXd& matrix, int precision = 4)
{
    if (matrix.rows() == 0 || matrix.cols() == 0)
        return "[]";

    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision);

    for (int i = 0; i < matrix.rows(); ++i)
    {
        oss << (i == 0 ? "[" : "; ");

        for (int j = 0; j < matrix.cols(); ++j)
        {
            if (j > 0)
                oss << " ";

            oss << matrix(i, j);
        }

        if (i + 1 == matrix.rows())
            oss << "]";
    }

    return oss.str();
}

} // namespace ins_ros::utils::gating
