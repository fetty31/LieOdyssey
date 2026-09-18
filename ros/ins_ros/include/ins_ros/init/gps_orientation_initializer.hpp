#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <cstddef>
#include <vector>

namespace ins_ros::init {

class GPSOrientationInitializer
{
public:

    struct Parameters
    {
        // Minimum distance traveled before estimating heading.
        double distance_threshold = 2.0;

        // Minimum distance between consecutive stored GPS points.
        double delta_distance_threshold = 0.1;

        // Maximum physically plausible robot speed.
        double max_speed = 20.5;

        // Minimum number of GPS points used by PCA.
        std::size_t min_samples = 3;
    };

    GPSOrientationInitializer()
        : params_()
    {
    }

    explicit GPSOrientationInitializer(
        const Parameters& params)
        : params_(params)
    {
    }

    void reset()
    {
        initialized_ = false;
        last_position_.setZero();
        path_segment_.clear();

        distance_traveled_ = 0.0;
        heading_ = 0.0;
        last_time_ = -1.0;
        last_dt_ = 0.0;
    }

    /**
     * Add an ENU GPS position.
     *
     * @param position_enu GPS position expressed in local ENU [m].
     * @param timestamp    Timestamp in seconds.
     *
     * @return true if the GPS heading has been initialized.
     */
    bool add_position(
        const Eigen::Vector3d& position_enu,
        double timestamp)
    {
        if (initialized_)
            return true;

        const Eigen::Vector2d current_position =
            position_enu.head<2>();

        // First position
        if (last_time_ < 0.0)
        {
            last_position_ = current_position;
            path_segment_.push_back(current_position);
            last_time_ = timestamp;
            return false;
        }

        // Compute displacement
        const Eigen::Vector2d delta =
            current_position - last_position_;

        const double delta_distance = delta.norm();

        // Validate GPS speed
        const double delta_time =
            timestamp - last_time_;

        if (delta_time <= 0.0){
            reset();
            return false;
        }

        const double speed_magnitude =
            delta_distance / delta_time;

        if (speed_magnitude > params_.max_speed)
        {
            // Don't update the reference point.
            // This measurement is considered invalid.
            reset();
            return false;
        }

        // Ignore very small movements
        if (delta_distance < params_.delta_distance_threshold)
            return false;

        // Update
        last_time_ = timestamp;
        last_dt_ = delta_time;

        distance_traveled_ += delta_distance;

        last_position_ = current_position;

        path_segment_.push_back(current_position);

        // Check whether enough data is available
        if (distance_traveled_ < params_.distance_threshold)
            return false;

        if (path_segment_.size() < params_.min_samples)
            return false;

        // Estimate heading using PCA
        if (!compute_pca_heading(heading_))
        {
            reset();
            return false;
        }

        initialized_ = true;

        return true;
    }

    bool initialized() const
    {
        return initialized_;
    }

    double heading() const
    {
        return heading_;
    }

    double velocity() const
    {
        if(path_segment_.size() < 2)
            return 0.0;
        
        if(last_dt_ <= 0.0)
            return 0.0;

        const Eigen::Vector2d forward =
            Eigen::Rotation2Dd(heading_) * Eigen::Vector2d::UnitX();

        const auto& p1 = path_segment_.back();
        const auto& p0 = path_segment_[path_segment_.size()-2];
        const Eigen::Vector2d delta = p1 - p0;

        return delta.dot(forward) / delta_time;
    }

    const double stamp() const 
    {
        return last_time_;
    }

    Eigen::Vector2d position() const
    {
        return last_position_;
    }

    Eigen::Quaterniond orientation() const
    {
        Eigen::Quaterniond q(
            Eigen::AngleAxisd(
                heading_,
                Eigen::Vector3d::UnitZ()));

        q.normalize();

        return q;
    }

private:

    bool compute_pca_heading(double& heading) const
    {
        const std::size_t N =
            path_segment_.size();

        Eigen::Matrix<double, Eigen::Dynamic, 2> points(
            static_cast<Eigen::Index>(N),
            2);

        for (std::size_t i = 0; i < N; ++i)
        {
            points.row(
                static_cast<Eigen::Index>(i)) =
                path_segment_[i].transpose();
        }

        // Remove centroid
        const Eigen::Vector2d centroid =
            points.colwise().mean();

        points.rowwise() -= centroid.transpose();

        // PCA covariance
        const Eigen::Matrix2d covariance =
            points.transpose() * points /
            static_cast<double>(N);

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(
            covariance);

        if (solver.info() != Eigen::Success)
            return false;

        // Eigenvalues are sorted in ascending order.
        // Therefore col(1) corresponds to the principal direction.
        Eigen::Vector2d principal_direction =
            solver.eigenvectors().col(1);

        // ---------------------------------------------------------
        // Resolve PCA's ± direction ambiguity
        //
        // PCA tells us the line direction but not whether the
        // robot moved along +v or -v.
        //
        // Use first -> last position to determine the actual
        // direction of motion.
        // ---------------------------------------------------------

        const Eigen::Vector2d motion =
            path_segment_.back() -
            path_segment_.front();

        if (motion.dot(principal_direction) < 0.0)
        {
            principal_direction = -principal_direction;
        }

        heading = std::atan2(
            principal_direction.y(),
            principal_direction.x());

        return true;
    }

public:

    Parameters params_;

    bool initialized_ = false;

    Eigen::Vector2d last_position_ =
        Eigen::Vector2d::Zero();

    double last_time_ = 0.0;
    double last_dt_ = 0.0;

    std::vector<Eigen::Vector2d> path_segment_;

    double distance_traveled_ = 0.0;

    double heading_ = 0.0;
};

} // namespace ins_ros::init