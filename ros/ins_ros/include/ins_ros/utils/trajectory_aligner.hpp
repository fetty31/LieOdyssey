#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>
#include <vector>

namespace ins_ros::utils {

class TrajectoryAligner
{
public:
    struct Pose
    {
        double time;
        Eigen::Vector3d position;
    };

    using Trajectory = std::vector<Pose>;

    struct SynchronizedTrajectories
    {
        Trajectory source;
        Trajectory target;
    };

    TrajectoryAligner() = default;

    void addSourcePose(Eigen::Vector3d position, double time)
    {
        if(aligned_) reset();
        source_.push_back({time, position});
    }

    void addSourcePose(const Pose& pose)
    {
        if(aligned_) reset();
        source_.push_back(pose);
    }

    void addTargetPose(Eigen::Vector3d position, double time)
    {
        if(aligned_) reset();
        target_.push_back({time, position});
    }

    void addTargetPose(const Pose& pose)
    {
        if(aligned_) reset();
        target_.push_back(pose);
    }

    void clear()
    {
        source_.clear();
        target_.clear();
    }

    void reset()
    {
        clear();
        aligned_ = false;
    }

    bool isAligned() const
    {
        return aligned_;
    }

    Trajectory getSourceTrajectory() const
    {
        return source_;
    }

    Trajectory getSourceTrajectory(const Eigen::Isometry3d& T) const
    {
        Trajectory transformed;
        transformed.reserve(source_.size());

        for (const auto& pose : source_)
        {
            transformed.push_back({
                pose.time,
                T * pose.position
            });
        }

        return transformed;
    }

    Trajectory getTargetTrajectory() const
    {
        return target_;
    }

    /**
     * Synchronize trajectories over their common temporal interval.
     *
     * Source timestamps are used as the reference timestamps and the target
     * trajectory is linearly interpolated at those timestamps.
     *
     * @return Synchronized source/target pose pairs.
     */
    SynchronizedTrajectories synchronize() const
    {
        SynchronizedTrajectories synchronized;

        if (source_.size() < 2 || target_.size() < 2)
            return synchronized;

        const double start_time =
            std::max(source_.front().time, target_.front().time);

        const double end_time =
            std::min(source_.back().time, target_.back().time);

        if (start_time >= end_time)
            return synchronized;

        for (const auto& source_pose : source_)
        {
            if (source_pose.time < start_time)
                continue;

            if (source_pose.time > end_time)
                break;

            Pose target_pose;

            if (!interpolate(
                    target_,
                    source_pose.time,
                    target_pose))
            {
                continue;
            }

            synchronized.source.push_back(source_pose);
            synchronized.target.push_back(target_pose);
        }

        return synchronized;
    }

    /**
     * Compute the total travelled distance of a trajectory.
     *
     * The distance is computed as the sum of Euclidean distances between
     * consecutive poses:
     *
     *     distance = sum ||p_i - p_{i-1}||
     *
     * @param trajectory Trajectory whose travelled distance is computed.
     *
     * @return Total travelled distance [m].
     */
    static double computeTotalDistance(const Trajectory& trajectory)
    {
        if (trajectory.size() < 2)
            return 0.0;

        double distance = 0.0;
        for (std::size_t i = 1; i < trajectory.size(); ++i)
        {
            distance +=
                (trajectory[i].position - trajectory[i - 1].position).norm();
        }
        return distance;
    }

    /**
     * Compute travelled distance of the synchronized source trajectory.
     *
     * @return Synchronized source trajectory distance [m].
     */
    double sourceDistance() const
    {
        const auto synchronized = synchronize();
        return computeTotalDistance(synchronized.source);
    }

    /**
     * Compute travelled distance of the synchronized target trajectory.
     *
     * @return Synchronized target trajectory distance [m].
     */
    double targetDistance() const
    {
        const auto synchronized = synchronize();
        return computeTotalDistance(synchronized.target);
    }

    /**
     * Compute travelled distances of both synchronized trajectories.
     *
     * @param source_distance Output source trajectory distance [m].
     * @param target_distance Output target trajectory distance [m].
     *
     * @return Number of synchronized poses.
     */
    std::size_t synchronizedDistances(
        double& source_distance,
        double& target_distance) const
    {
        const auto synchronized = synchronize();

        source_distance = computeTotalDistance(synchronized.source);
        target_distance = computeTotalDistance(synchronized.target);

        return synchronized.source.size();
    }

    /**
     * Align trajectory source to trajectory target using Umeyama.
     *
     * The returned transform satisfies approximately:
     *
     *     p_target = T_source_to_target * p_source
     *
     * @param travelled_dist Minimum travelled distance in order to align [m].
     * @param transform Output rigid transform.
     *
     * @return true if enough corresponding poses were found.
     */
    bool align(
        double travelled_dist,
        Eigen::Isometry3d& transform)
    {
        const auto synchronized = synchronize();

        if (synchronized.source.size() < 3)
            return false;

        const double source_distance = computeTotalDistance(synchronized.source);
        const double target_distance = computeTotalDistance(synchronized.target);

        if ( (source_distance < travelled_dist) || (target_distance < travelled_dist) )
            return false;

        Eigen::Matrix<double, 3, Eigen::Dynamic> source_matrix(
            3, synchronized.source.size());

        Eigen::Matrix<double, 3, Eigen::Dynamic> target_matrix(
            3, synchronized.target.size());

        const Eigen::Vector3d source_origin =
            synchronized.source.front().position;

        const Eigen::Vector3d target_origin =
            synchronized.target.front().position;

        for (std::size_t i = 0; i < synchronized.source.size(); ++i)
        {
            source_matrix.col(i) =
                synchronized.source[i].position - source_origin;

            target_matrix.col(i) =
                synchronized.target[i].position - target_origin;
        }

        const Eigen::Matrix4d matrix =
            Eigen::umeyama(source_matrix, target_matrix, false);

        const Eigen::Matrix3d R = matrix.block<3, 3>(0, 0);

        const Eigen::Vector3d t = target_origin - R * source_origin;

        transform = Eigen::Isometry3d::Identity();
        transform.linear() = R;
        transform.translation() = t;

        aligned_ = true;

        return true;
    }

private:
    bool interpolate(
        const Trajectory& trajectory,
        double time,
        Pose& pose) const
    {
        if (trajectory.size() < 2)
            return false;

        // Outside trajectory range.
        if (time < trajectory.front().time ||
            time > trajectory.back().time)
        {
            return false;
        }

        // Find first pose with time >= requested time.
        auto it = std::lower_bound(
            trajectory.begin(),
            trajectory.end(),
            time,
            [](const Pose& p, double t)
            {
                return p.time < t;
            });

        // Exact match.
        if (it != trajectory.end() && it->time == time)
        {
            pose = *it;
            return true;
        }

        // Need one point before and one after.
        if (it == trajectory.begin() || it == trajectory.end())
            return false;

        const Pose& p1 = *(it - 1);
        const Pose& p2 = *it;

        const double dt = p2.time - p1.time;

        if (dt <= 0.0)
            return false;

        const double alpha =
            (time - p1.time) / dt;

        pose.time = time;
        pose.position =
            (1.0 - alpha) * p1.position +
            alpha * p2.position;

        return true;
    }


private:

    Trajectory source_;
    Trajectory target_;

    bool aligned_ = false;

};

} // namespace ins_ros::utils