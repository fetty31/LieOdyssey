#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>

#include <string>

namespace ins_ros::utils {

class FrameTransform
{
public:

    using Scalar = double;
    using Vector3 = Eigen::Vector3d;
    using Matrix3 = Eigen::Matrix3d;
    using Quaternion = Eigen::Quaterniond;

    FrameTransform() = default;

    FrameTransform(
        tf2_ros::Buffer& tf_buffer,
        rclcpp::Logger logger)
        : tf_buffer_(tf_buffer),
          logger_(logger)
    {
    }

    /**
     * Initialize the transform:
     *
     *     source_frame -> target_frame
     *
     * The resulting transform satisfies:
     *
     *     v_target = R_target_source * v_source
     *
     *     p_target = R_target_source * p_source
     *                + t_target_source
     */
    bool initialize(
        const std::string& source_frame,
        const std::string& target_frame)
    {
        if (source_frame.empty() || source_frame == target_frame)
        {
            if (source_frame.empty())
            {
                RCLCPP_WARN(
                    logger_,
                    "Source frame is empty. Assuming '%s' frame.",
                    target_frame.c_str());
            }else
            {
                RCLCPP_WARN(
                    logger_,
                    "Source frame '%s' is the same as target frame. Assuming identity transform.",
                    source_frame.c_str());
            }

            R_.setIdentity();
            t_.setZero();

            initialized_ = true;

            return true;
        }

        try
        {
            const auto tf = tf_buffer_.lookupTransform(
                target_frame,
                source_frame,
                tf2::TimePointZero,
                tf2::durationFromSec(1.0));

            const auto& q = tf.transform.rotation;

            Quaternion quat(
                q.w,
                q.x,
                q.y,
                q.z);

            quat.normalize();

            R_ = quat.toRotationMatrix();

            const auto& t = tf.transform.translation;

            t_ = Vector3(
                t.x,
                t.y,
                t.z);

            initialized_ = true;

            RCLCPP_INFO(
                logger_,
                "Initialized transform: %s -> %s",
                source_frame.c_str(),
                target_frame.c_str());

            return true;
        }
        catch (const tf2::TransformException& ex)
        {
            RCLCPP_ERROR(
                logger_,
                "Failed to initialize transform from '%s' to '%s': %s",
                source_frame.c_str(),
                target_frame.c_str(),
                ex.what());

            return false;
        }
    }

    void reset()
    {
        initialized_ = false;
        R_.setIdentity();
        t_.setZero();
    }

    bool initialized() const
    {
        return initialized_;
    }

    const Matrix3& rotation() const
    {
        return R_;
    }

    const Vector3& translation() const
    {
        return t_;
    }

    /**
     * Rotate a vector from source to target.
     *
     * No translation is applied.
     */
    Vector3 rotate(
        const Vector3& vector) const
    {
        return R_ * vector;
    }

    /*
     * Transform a vector from source frame to target frame.
     *
     *     p_target = R_target_source * p_source + t_target_source
     */
    Vector3 transform(
        const Vector3& p) const
    {
        return R_ * p + t_;
    }

    /**
     * Transform state from source to target.
     */
    ins_ros::State transform(const ins_ros::State& state) const
    {
        ins_ros::State transformed = state;

        // Position
        transformed.p =
            R_ * state.p + t_;

        // Orientation
        transformed.q =
            Quaternion(R_) * state.q;

        transformed.q.normalize();

        return transformed;
    }

private:

    tf2_ros::Buffer& tf_buffer_;

    rclcpp::Logger logger_;

    bool initialized_{false};

    Matrix3 R_ =
        Matrix3::Identity();

    Vector3 t_ =
        Vector3::Zero();
};

} // namespace ins_ros::utils