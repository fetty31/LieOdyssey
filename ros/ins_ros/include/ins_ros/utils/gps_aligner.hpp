#pragma once

#include "TrajectoryAligner.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ins_ros::utils {

class GPSAligner : public TrajectoryAligner
{
public:

    struct GPSDiagnostics : public Diagnostics
    {
        std::size_t ransac_iterations = 0;
        double inlier_ratio = 0.0;
    };

    struct Config
    {
        // Minimum horizontal distance between RANSAC sample points.
        double min_pair_baseline = 2.0;

        // Mahalanobis distance threshold for a 3D residual.
        double mahalanobis_threshold = 7.815;

        // Minimum number of RANSAC inliers.
        std::size_t min_inliers = 5;

        std::size_t max_ransac_iterations = 500;

        std::size_t max_optimization_iterations = 20;

        double convergence_threshold = 1e-8;

        // Additional uncertainty for numerical stability.
        double covariance_regularization = 1e-6;
    };

    explicit GPSAligner(const Config& config = Config{})
        : config_(config)
    {
    }

    using TrajectoryAligner::align;

    // ---------------------------------------------------------------------
    // 4DoF alignment
    // ---------------------------------------------------------------------

    bool align(
        double max_interpolation_gap,
        AlignmentResult& result,
        Diagnostics* diagnostics = nullptr) override
    {
        result.reset();

        GPSDiagnostics gps_diagnostics;

        const auto synchronized =
            synchronize(max_interpolation_gap);

        gps_diagnostics.num_correspondences = synchronized.size();

        if ( (synchronized.size() < config_.min_inliers) || 
            (!hasSufficientDistance(synchronized)) )
        {
            if (diagnostics)
                *diagnostics = gps_diagnostics;

            return false;
        }

        // -------------------------------------------------------------
        // 1. RANSAC initialization
        // -------------------------------------------------------------

        Eigen::Matrix<double, 4, 1> best_parameters =
            Eigen::Matrix<double, 4, 1>::Zero();

        std::vector<std::size_t> best_inliers;

        std::size_t iterations = 0;

        const std::size_t N = synchronized.size();

        for (std::size_t i = 0;
             i < N && iterations < config_.max_ransac_iterations;
             ++i)
        {
            for (std::size_t j = i + 1;
                 j < N && iterations < config_.max_ransac_iterations;
                 ++j)
            {
                ++iterations;

                const auto& a = synchronized[i];
                const auto& b = synchronized[j];

                const Eigen::Vector2d ds =
                    (b.source.position - a.source.position).head<2>();

                const Eigen::Vector2d dt =
                    (b.target.position - a.target.position).head<2>();

                if (ds.norm() < config_.min_pair_baseline ||
                    dt.norm() < config_.min_pair_baseline)
                    continue;

                const double cross =
                    ds.x() * dt.y() - ds.y() * dt.x();

                const double dot = ds.dot(dt);

                const double yaw = std::atan2(cross, dot);

                const Eigen::Matrix3d R = rotationMatrix(yaw);

                const Eigen::Vector3d translation =
                    0.5 * (
                        a.target.position - R * a.source.position +
                        b.target.position - R * b.source.position);

                Eigen::Matrix<double, 4, 1> candidate;
                candidate << yaw,
                             translation.x(),
                             translation.y(),
                             translation.z();

                const auto inliers =
                    classifyInliers(synchronized, candidate);

                if (inliers.size() > best_inliers.size())
                {
                    best_inliers = inliers;
                    best_parameters = candidate;
                }
            }
        }

        gps_diagnostics.ransac_iterations = iterations;

        if (best_inliers.size() < config_.min_inliers)
        {
            if (diagnostics)
                *diagnostics = gps_diagnostics;

            return false;
        }

        // -------------------------------------------------------------
        // 2. Covariance-weighted nonlinear optimization
        // -------------------------------------------------------------

        Eigen::Matrix<double, 4, 4> H =
            Eigen::Matrix<double, 4, 4>::Zero();

        bool converged = false;

        for (std::size_t iteration = 0;
             iteration < config_.max_optimization_iterations;
             ++iteration)
        {
            H.setZero();

            Eigen::Matrix<double, 4, 1> b =
                Eigen::Matrix<double, 4, 1>::Zero();

            const double yaw = best_parameters(0);
            const Eigen::Matrix3d R = rotationMatrix(yaw);

            for (const auto index : best_inliers)
            {
                const auto& pair = synchronized[index];

                const Eigen::Vector3d q =
                    R * pair.source.position +
                    best_parameters.tail<3>();

                const Eigen::Vector3d residual =
                    pair.target.position - q;

                const Eigen::Matrix3d covariance =
                    residualCovariance(pair, R);

                Eigen::LDLT<Eigen::Matrix3d> solver(covariance);

                if (solver.info() != Eigen::Success)
                    return false;

                const Eigen::Matrix3d information =
                    solver.solve(Eigen::Matrix3d::Identity());

                Eigen::Matrix<double, 3, 4> J;

                // Derivative of residual w.r.t. yaw.
                const double c = std::cos(yaw);
                const double s = std::sin(yaw);

                const double x = pair.source.position.x();
                const double y = pair.source.position.y();

                J.col(0) << s * x + c * y,
                            -c * x + s * y,
                            0.0;

                J.block<3, 3>(0, 1) =
                    -Eigen::Matrix3d::Identity();

                H.noalias() += J.transpose() * information * J;
                b.noalias() += J.transpose() * information * residual;
            }

            Eigen::LDLT<Eigen::Matrix<double, 4, 4>> solver(H);

            if (solver.info() != Eigen::Success)
                return false;

            const Eigen::Matrix<double, 4, 1> delta =
                -solver.solve(b);

            if (!delta.allFinite())
                return false;

            best_parameters += delta;

            // Keep yaw in [-pi, pi].
            best_parameters(0) =
                std::atan2(
                    std::sin(best_parameters(0)),
                    std::cos(best_parameters(0)));

            if (delta.norm() < config_.convergence_threshold)
            {
                converged = true;
                break;
            }
        }

        if (!converged)
        {
            // A final update may have been sufficiently close even if
            // the iteration limit was reached.
            if (best_inliers.size() < config_.min_inliers)
                return false;
        }

        // -------------------------------------------------------------
        // 3. Recompute final covariance and diagnostics
        // -------------------------------------------------------------

        H.setZero();

        double squared_error = 0.0;

        const double yaw = best_parameters(0);
        const Eigen::Matrix3d R = rotationMatrix(yaw);

        for (const auto index : best_inliers)
        {
            const auto& pair = synchronized[index];

            const Eigen::Vector3d q =
                R * pair.source.position +
                best_parameters.tail<3>();

            const Eigen::Vector3d residual =
                pair.target.position - q;

            const Eigen::Matrix3d covariance =
                residualCovariance(pair, R);

            Eigen::LDLT<Eigen::Matrix3d> solver(covariance);

            if (solver.info() != Eigen::Success)
                return false;

            const Eigen::Matrix3d information =
                solver.solve(Eigen::Matrix3d::Identity());

            Eigen::Matrix<double, 3, 4> J;

            const double c = std::cos(yaw);
            const double s = std::sin(yaw);

            const double x = pair.source.position.x();
            const double y = pair.source.position.y();

            J.col(0) << s * x + c * y,
                        -c * x + s * y,
                        0.0;

            J.block<3, 3>(0, 1) =
                -Eigen::Matrix3d::Identity();

            H.noalias() += J.transpose() * information * J;

            squared_error += residual.squaredNorm();
        }

        Eigen::LDLT<Eigen::Matrix<double, 4, 4>> solver(H);

        if (solver.info() != Eigen::Success)
            return false;

        const Eigen::Matrix<double, 4, 4> covariance =
            solver.solve(Eigen::Matrix4d::Identity());

        if (!covariance.allFinite())
            return false;

        // -------------------------------------------------------------
        // 4. Build the 4DoF result
        // -------------------------------------------------------------

        Eigen::Isometry3d transform =
            Eigen::Isometry3d::Identity();

        transform.linear() = R;
        transform.translation() = best_parameters.tail<3>();

        result.transform = transform;
        result.covariance = covariance;
        result.dof = 4;
        result.valid = true;

        aligned_ = true;

        gps_diagnostics.num_inliers = best_inliers.size();
        gps_diagnostics.num_outliers =
            synchronized.size() - best_inliers.size();

        gps_diagnostics.inlier_ratio =
            static_cast<double>(best_inliers.size()) /
            synchronized.size();

        gps_diagnostics.rmse =
            std::sqrt(squared_error / best_inliers.size());

        gps_diagnostics.valid = true;

        if (diagnostics)
            *diagnostics = gps_diagnostics;

        return true;
    }

    // ---------------------------------------------------------------------
    // GPS-specific distance metric
    // ---------------------------------------------------------------------

    double distanceBetween(
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b) const override
    {
        return (a.head<2>() - b.head<2>()).norm();
    }

private:

    Config config_;

    // ---------------------------------------------------------------------
    // Geometry
    // ---------------------------------------------------------------------

    static Eigen::Matrix3d rotationMatrix(double yaw)
    {
        const double c = std::cos(yaw);
        const double s = std::sin(yaw);

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

        R(0, 0) = c;
        R(0, 1) = -s;
        R(1, 0) = s;
        R(1, 1) = c;

        return R;
    }

    Eigen::Matrix3d residualCovariance(
        const Correspondence& pair,
        const Eigen::Matrix3d& R) const
    {
        Eigen::Matrix3d covariance =
            pair.target.covariance +
            R * pair.source.covariance * R.transpose();

        covariance =
            0.5 * (covariance + covariance.transpose());

        covariance.diagonal().array() +=
            config_.covariance_regularization;

        return covariance;
    }

    // ---------------------------------------------------------------------
    // Outlier rejection
    // ---------------------------------------------------------------------

    std::vector<std::size_t> classifyInliers(
        const std::vector<Correspondence>& correspondences,
        const Eigen::Matrix<double, 4, 1>& parameters) const
    {
        std::vector<std::size_t> inliers;

        const double yaw = parameters(0);
        const Eigen::Matrix3d R = rotationMatrix(yaw);

        for (std::size_t i = 0; i < correspondences.size(); ++i)
        {
            const auto& pair = correspondences[i];

            const Eigen::Vector3d predicted =
                R * pair.source.position +
                parameters.tail<3>();

            const Eigen::Vector3d residual =
                pair.target.position - predicted;

            const Eigen::Matrix3d covariance =
                residualCovariance(pair, R);

            Eigen::LDLT<Eigen::Matrix3d> solver(covariance);

            if (solver.info() != Eigen::Success)
                continue;

            const double mahalanobis =
                residual.dot(solver.solve(residual));

            if (mahalanobis < config_.mahalanobis_threshold)
                inliers.push_back(i);
        }

        return inliers;
    }
};

} // namespace ins_ros::utils