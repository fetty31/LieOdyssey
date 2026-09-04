#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>

namespace ins_ros::init {

class IMUOrientationInitializer
{
public:

    struct Parameters
    {
        double gravity = 9.81;
        double gravity_tolerance = 0.5;
        double gyro_stationary_threshold = 0.1;
        std::size_t min_samples = 200;
    };

    IMUOrientationInitializer()
        : params_()
    {
    }

    explicit IMUOrientationInitializer(
        const Parameters& params)
        : params_(params)
    {
    }

    void reset()
    {
        initialized_ = false;

        sample_count_ = 0;

        accel_sum_.setZero();
        gyro_sum_.setZero();

        accel_bias_.setZero();
        gyro_bias_.setZero();

        orientation_ =
            Eigen::Quaterniond::Identity();
    }

    bool add_measurement(
        const Eigen::Vector3d& accel,
        const Eigen::Vector3d& gyro)
    {
        if (initialized_)
            return true;

        // Check acceleration magnitude
        if (std::abs(
                accel.norm() - params_.gravity)
            > params_.gravity_tolerance)
        {
            reset_accumulation();
            return false;
        }

        // Check angular velocity
        if (gyro.norm() >
            params_.gyro_stationary_threshold)
        {
            reset_accumulation();
            return false;
        }

        // Accumulate measurements
        accel_sum_ += accel;
        gyro_sum_ += gyro;

        ++sample_count_;

        if (sample_count_ < params_.min_samples)
            return false;

        // Compute means
        const Eigen::Vector3d accel_mean = mean_acceleration();
        const Eigen::Vector3d gyro_mean = mean_gyro();

        // Estimate roll/pitch
        orientation_ =
            compute_roll_pitch(accel_mean);

        // Accelerometer bias
        //
        // At rest:
        //
        //   a_meas = b_a + [0, 0, g]
        //
        // assuming the robot is level.
        accel_bias_ =
            accel_mean -
            Eigen::Vector3d(
                0.0,
                0.0,
                params_.gravity);

        // Gyroscope bias
        gyro_bias_ = gyro_mean;

        initialized_ = true;

        return true;
    }

    bool initialized() const
    {
        return initialized_;
    }

    const Eigen::Quaterniond& orientation() const
    {
        return orientation_;
    }

    Eigen::Matrix3d rotation() const
    {
        return orientation_.toRotationMatrix();
    }

    Eigen::Vector3d accelerometer_bias() const
    {
        return accel_bias_;
    }

    Eigen::Vector3d gyroscope_bias() const
    {
        return gyro_bias_;
    }

    Eigen::Vector3d mean_acceleration() const
    {
        if (sample_count_ == 0)
            return Eigen::Vector3d::Zero();

        return accel_sum_ /
               static_cast<double>(sample_count_);
    }

    Eigen::Vector3d mean_gyro() const
    {
        if (sample_count_ == 0)
            return Eigen::Vector3d::Zero();

        return gyro_sum_ /
               static_cast<double>(sample_count_);
    }

    Eigen::Quaterniond compute_roll_pitch(
        const Eigen::Vector3d& accel) const
    {
        const double ax = accel.x();
        const double ay = accel.y();
        const double az = accel.z();

        const double roll =
            std::atan2(ay, az);

        const double pitch =
            std::atan2(
                -ax,
                std::sqrt(
                    ay * ay +
                    az * az));

        Eigen::AngleAxisd roll_rotation(
            roll,
            Eigen::Vector3d::UnitX());

        Eigen::AngleAxisd pitch_rotation(
            pitch,
            Eigen::Vector3d::UnitY());

        Eigen::Quaterniond q =
            pitch_rotation * roll_rotation;

        q.normalize();

        return q;
    }

private:

    void reset_accumulation()
    {
        sample_count_ = 0;

        accel_sum_.setZero();
        gyro_sum_.setZero();
    }

public:

    Parameters params_;

    bool initialized_ = false;

    std::size_t sample_count_ = 0;

    Eigen::Vector3d accel_sum_ =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d gyro_sum_ =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d accel_bias_ =
        Eigen::Vector3d::Zero();

    Eigen::Vector3d gyro_bias_ =
        Eigen::Vector3d::Zero();

    Eigen::Quaterniond orientation_ =
        Eigen::Quaterniond::Identity();
};

} // namespace ins_ros::init