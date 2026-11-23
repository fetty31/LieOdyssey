#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <Eigen/Cholesky>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace ins_ros::utils {

class TrajectoryAligner
{
public:
    struct Pose
    {
        double time = 0.0;
        Eigen::Vector3d position = Eigen::Vector3d::Zero();
        Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    };

    using Trajectory = std::vector<Pose>;

    struct AlignmentResult
    {
        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();

        // 6x6 for SE(3), 4x4 for 4DoF GPS alignment.
        Eigen::MatrixXd covariance;

        // Number of estimated parameters.
        std::size_t dof = 0;

        bool valid = false;

        void reset()
        {
            transform.setIdentity();
            covariance.resize(0, 0);
            dof = 0;
            valid = false;
        }
    };

    struct Diagnostics
    {
        std::size_t num_correspondences = 0;
        std::size_t num_inliers = 0;
        std::size_t num_outliers = 0;

        double rmse = std::numeric_limits<double>::quiet_NaN();

        bool valid = false;
    };

    struct SynchronizedTrajectories
    {
        Trajectory source;
        Trajectory target;

        std::size_t size() const
        {
            return source.size();
        }

        bool empty() const
        {
            return source.empty();
        }
    };

    static constexpr std::size_t MAX_CAPACITY = 10000;

    virtual ~TrajectoryAligner() = default;

    // ---------------------------------------------------------------------
    // Trajectory management
    // ---------------------------------------------------------------------

    bool addSourcePose(
        double time,
        const Eigen::Vector3d& position,
        const Eigen::Matrix3d& covariance = Eigen::Matrix3d::Zero())
    {
        return addPose(source_, time, position, covariance);
    }

    bool addTargetPose(
        double time,
        const Eigen::Vector3d& position,
        const Eigen::Matrix3d& covariance = Eigen::Matrix3d::Zero())
    {
        return addPose(target_, time, position, covariance);
    }

    void clear()
    {
        source_.clear();
        target_.clear();
        aligned_ = false;
    }

    void reset()
    {
        clear();
    }

    void setMinimumDistance(double distance)
    {
        min_distance_ = std::max(0.0, distance);
    }

    double minimumDistance() const
    {
        return min_distance_;
    }

    bool isAligned() const
    {
        return aligned_;
    }

    const Trajectory& sourceTrajectory() const { return source_; }
    const Trajectory& targetTrajectory() const { return target_; }

    // ---------------------------------------------------------------------
    // Alignment interface
    // ---------------------------------------------------------------------

    virtual bool align(
        double max_interpolation_gap,
        AlignmentResult& result,
        Diagnostics* diagnostics = nullptr)
    {
        result.reset();

        if (diagnostics)
            *diagnostics = Diagnostics{};

        const auto synchronized = synchronize(max_interpolation_gap);

        if (diagnostics)
            diagnostics->num_correspondences = synchronized.size();

        if (synchronized.size() < 3)
            return false;

        if (!hasSufficientDistance(synchronized))
            return false;

        // Initial estimate using Umeyama.
        Eigen::Matrix<double, 3, Eigen::Dynamic> source(3, synchronized.size());
        Eigen::Matrix<double, 3, Eigen::Dynamic> target(3, synchronized.size());

        for (std::size_t i = 0; i < synchronized.size(); ++i)
        {
            source.col(i) = synchronized[i].source.position;
            target.col(i) = synchronized[i].target.position;
        }

        const Eigen::Matrix4d initial =
            Eigen::umeyama(source, target, false);

        Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
        transform.matrix() = initial;

        // Refine using covariance-weighted least squares.
        Eigen::Matrix<double, 6, 6> H =
            Eigen::Matrix<double, 6, 6>::Zero();

        for (int iteration = 0; iteration < 20; ++iteration)
        {
            H.setZero();

            Eigen::Matrix<double, 6, 1> b =
                Eigen::Matrix<double, 6, 1>::Zero();

            for (const auto& pair : synchronized)
            {
                const Eigen::Vector3d q =
                    transform * pair.source.position;

                const Eigen::Vector3d residual =
                    pair.target.position - q;

                const Eigen::Matrix3d covariance =
                    regularizeCovariance(
                        pair.target.covariance +
                        transform.linear() *
                            pair.source.covariance *
                            transform.linear().transpose());

                Eigen::LDLT<Eigen::Matrix3d> solver(covariance);

                if (solver.info() != Eigen::Success)
                    return false;

                const Eigen::Matrix3d information =
                    solver.solve(Eigen::Matrix3d::Identity());

                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = skew(q);
                J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

                H.noalias() += J.transpose() * information * J;
                b.noalias() += J.transpose() * information * residual;
            }

            Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H);

            if (solver.info() != Eigen::Success)
                return false;

            const Eigen::Matrix<double, 6, 1> delta =
                -solver.solve(b);

            if (!delta.allFinite())
                return false;

            transform = expSE3(delta) * transform;

            if (delta.norm() < 1e-8)
                break;
        }

        // Recompute the Hessian at the final estimate.
        H.setZero();

        double squared_error = 0.0;

        for (const auto& pair : synchronized)
        {
            const Eigen::Vector3d q =
                transform * pair.source.position;

            const Eigen::Vector3d residual =
                pair.target.position - q;

            const Eigen::Matrix3d covariance =
                regularizeCovariance(
                    pair.target.covariance +
                    transform.linear() *
                        pair.source.covariance *
                        transform.linear().transpose());

            Eigen::LDLT<Eigen::Matrix3d> solver(covariance);

            if (solver.info() != Eigen::Success)
                return false;

            const Eigen::Matrix3d information =
                solver.solve(Eigen::Matrix3d::Identity());

            Eigen::Matrix<double, 3, 6> J;
            J.block<3, 3>(0, 0) = skew(q);
            J.block<3, 3>(0, 3) = -Eigen::Matrix3d::Identity();

            H.noalias() += J.transpose() * information * J;

            squared_error += residual.squaredNorm();
        }

        Eigen::LDLT<Eigen::Matrix<double, 6, 6>> solver(H);

        if (solver.info() != Eigen::Success)
            return false;

        result.transform = transform;
        result.covariance = solver.solve(
            Eigen::Matrix<double, 6, 6>::Identity());
        result.dof = 6;
        result.valid = true;

        aligned_ = true;

        if (diagnostics)
        {
            diagnostics->num_inliers = synchronized.size();
            diagnostics->rmse =
                std::sqrt(squared_error / synchronized.size());
            diagnostics->valid = true;
        }

        return true;
    }

    // Convenience interface. The virtual overload above is still dispatched.
    bool align(
        double max_interpolation_gap,
        Eigen::Isometry3d& transform)
    {
        AlignmentResult result;

        if (!align(max_interpolation_gap, result))
            return false;

        transform = result.transform;
        return true;
    }

    bool align(
        double max_interpolation_gap,
        Eigen::Isometry3d& transform,
        Eigen::MatrixXd& covariance)
    {
        AlignmentResult result;

        if (!align(max_interpolation_gap, result))
            return false;

        transform = result.transform;
        covariance = result.covariance;
        return true;
    }

    // ---------------------------------------------------------------------
    // Distance calculations
    // ---------------------------------------------------------------------

    virtual double distanceBetween(
        const Eigen::Vector3d& a,
        const Eigen::Vector3d& b) const
    {
        return (a - b).norm();
    }

    virtual double computeTotalDistance(
        const Trajectory& trajectory) const
    {
        double distance = 0.0;

        for (std::size_t i = 1; i < trajectory.size(); ++i)
        {
            distance += distanceBetween(
                trajectory[i - 1].position,
                trajectory[i].position);
        }

        return distance;
    }

    double sourceDistance() const
    {
        return computeTotalDistance(source_);
    }

    double targetDistance() const
    {
        return computeTotalDistance(target_);
    }

    SynchronizedTrajectories synchronizeTrajectories(
        double max_interpolation_gap) const
    {
        SynchronizedTrajectories result;

        if (source_.empty() || target_.size() < 2)
            return result;

        result.source.reserve(source_.size());
        result.target.reserve(source_.size());

        for (const auto& source_pose : source_)
        {
            Pose target_pose;

            if (!interpolate(
                    target_,
                    source_pose.time,
                    max_interpolation_gap,
                    target_pose))
                continue;

            result.source.push_back(source_pose);
            result.target.push_back(target_pose);
        }

        return result;
    }

protected:

    struct Correspondence
    {
        Pose source;
        Pose target;
    };

    Trajectory source_;
    Trajectory target_;

    bool aligned_ = false;

    double min_distance_ = 1.0;  // metres

    // ---------------------------------------------------------------------
    // Trajectory insertion
    // ---------------------------------------------------------------------

    bool addPose(
        Trajectory& trajectory,
        double time,
        const Eigen::Vector3d& position,
        const Eigen::Matrix3d& covariance)
    {
        if (!std::isfinite(time) ||
            !position.allFinite() ||
            !covariance.allFinite())
            return false;

        if (trajectory.size() >= MAX_CAPACITY)
            return false;

        // lower_bound and interpolation require sorted timestamps.
        if (!trajectory.empty() &&
            time <= trajectory.back().time)
            return false;

        Pose pose;
        pose.time = time;
        pose.position = position;
        pose.covariance = regularizeCovariance(covariance);

        trajectory.push_back(pose);
        aligned_ = false;

        return true;
    }

    // ---------------------------------------------------------------------
    // Synchronization and interpolation
    // ---------------------------------------------------------------------

    bool interpolate(
        const Trajectory& trajectory,
        double time,
        double max_gap,
        Pose& result) const
    {
        if (trajectory.size() < 2)
            return false;

        auto upper = std::lower_bound(
            trajectory.begin(),
            trajectory.end(),
            time,
            [](const Pose& pose, double t)
            {
                return pose.time < t;
            });

        if (upper == trajectory.begin() ||
            upper == trajectory.end())
            return false;

        const Pose& p1 = *(upper - 1);
        const Pose& p2 = *upper;

        const double dt = p2.time - p1.time;

        if (dt <= 0.0 || dt > max_gap)
            return false;

        const double alpha = (time - p1.time) / dt;

        result.time = time;

        result.position =
            (1.0 - alpha) * p1.position +
            alpha * p2.position;

        // Independent endpoint uncertainty propagation.
        result.covariance =
            (1.0 - alpha) * (1.0 - alpha) * p1.covariance +
            alpha * alpha * p2.covariance;

        result.covariance =
            regularizeCovariance(result.covariance);

        return true;
    }

    std::vector<Correspondence> synchronize(
        double max_interpolation_gap) const
    {
        std::vector<Correspondence> result;

        if (source_.empty() || target_.size() < 2)
            return result;

        result.reserve(source_.size());

        for (const auto& source_pose : source_)
        {
            Pose target_pose;

            if (!interpolate(
                    target_,
                    source_pose.time,
                    max_interpolation_gap,
                    target_pose))
                continue;

            result.push_back({source_pose, target_pose});
        }

        return result;
    }

    // ---------------------------------------------------------------------
    // Minimum displacement check after sync
    // ---------------------------------------------------------------------

    bool hasSufficientDistance(
        const std::vector<Correspondence>& correspondences) const
    {
        if (correspondences.size() < 2)
            return false;

        double source_distance = 0.0;
        double target_distance = 0.0;

        for (std::size_t i = 1; i < correspondences.size(); ++i)
        {
            source_distance += distanceBetween(
                correspondences[i - 1].source.position,
                correspondences[i].source.position);

            target_distance += distanceBetween(
                correspondences[i - 1].target.position,
                correspondences[i].target.position);
        }

        return source_distance >= min_distance_ &&
            target_distance >= min_distance_;
    }

    // ---------------------------------------------------------------------
    // Mathematical utilities
    // ---------------------------------------------------------------------

    static Eigen::Matrix3d skew(const Eigen::Vector3d& v)
    {
        Eigen::Matrix3d S;

        S << 0.0, -v.z(), v.y(),
             v.z(), 0.0, -v.x(),
            -v.y(), v.x(), 0.0;

        return S;
    }

    static Eigen::Matrix3d regularizeCovariance(
        const Eigen::Matrix3d& covariance,
        double epsilon = 1e-8)
    {
        Eigen::Matrix3d result =
            0.5 * (covariance + covariance.transpose());

        result.diagonal().array() += epsilon;

        return result;
    }

    static Eigen::Isometry3d expSE3(
        const Eigen::Matrix<double, 6, 1>& xi)
    {
        const Eigen::Vector3d w = xi.head<3>();
        const Eigen::Vector3d v = xi.tail<3>();

        const double theta = w.norm();

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d V = Eigen::Matrix3d::Identity();

        const Eigen::Matrix3d W = skew(w);
        const Eigen::Matrix3d W2 = W * W;

        if (theta < 1e-8)
        {
            R += W + 0.5 * W2;
            V += 0.5 * W + (1.0 / 6.0) * W2;
        }
        else
        {
            R +=
                (std::sin(theta) / theta) * W +
                ((1.0 - std::cos(theta)) / (theta * theta)) * W2;

            V +=
                ((1.0 - std::cos(theta)) / (theta * theta)) * W +
                ((theta - std::sin(theta)) /
                 (theta * theta * theta)) * W2;
        }

        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        T.linear() = R;
        T.translation() = V * v;

        return T;
    }
};

} // namespace ins_ros::utils