#pragma once

// Project includes
#include "ins_ros/state.hpp"
#include "ins_ros/ekf.hpp"

#include "ins_ros/sensors/baro_handler.hpp"
#include "ins_ros/sensors/gps_handler.hpp"
#include "ins_ros/sensors/mag_handler.hpp"
#include "ins_ros/sensors/pose_handler.hpp"
#include "ins_ros/sensors/wheel_handler.hpp"

// ROS
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>

#include <rclcpp/rclcpp.hpp>

// Messages
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

// TF
#include <tf2_ros/transform_broadcaster.h>

// Utilities
#include <memory>
#include <string>
#include <vector>

#include <boost/circular_buffer.hpp>

#include <Eigen/Geometry>

namespace ins_ros {

class INSEstimator : public rclcpp_lifecycle::LifecycleNode
{

    // TYPES

    public:

        using CallbackReturn =
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    // FUNCTIONS

    public:

        explicit INSEstimator(const std::string& node_name = "ins_ros_node");
        ~INSEstimator() override = default;

        // --- Lifecycle transitions ---
        CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
        CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
        CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
        CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;
        CallbackReturn on_error(const rclcpp_lifecycle::State& state) override;

    private:

        void imu_callback(const sensor_msgs::msg::Imu& msg);
        void gps_callback(const sensor_msgs::msg::NavSatFix& msg);
        void wheel_odom_callback(const geometry_msgs::msg::TwistStamped& msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped& msg);
        void mag_callback(const sensor_msgs::msg::MagneticField& msg);
        void baro_callback(const sensor_msgs::msg::FluidPressure& msg);

        // --- Init Filter ----
        void initState();

        // --- Automatic calibration utils ---
        // To-Do: add functions for online IMU bias estimation, gravity estimation, etc

        // --- Setup helpers (called during configure) ---
        void load_parameters();
        void setup_subscriptions();
        void setup_publishers();

        // --- ROS <-> Library conversion helpers ---
        void from_ros_to_ins(const sensor_msgs::msg::Imu& in, iESEKF::IMUmeas& out);
        void from_ros_to_ins(const geometry_msgs::msg::PoseStamped& in, ins_ros::State& out);
        void from_ins_to_ros(const ins_ros::State& in, nav_msgs::msg::Odometry& out);
        void from_ins_to_ros(const ins_ros::State& in, geometry_msgs::msg::PoseWithCovarianceStamped& out);
        void broadcast_tf(const ins_ros::State& in, bool now = true);

    // VARIABLES

    private:

        // EKF
        iESEKF::Filter filter_;

        // State
        ins_ros::State state_;

        // Buffers
        boost::circular_buffer<ins_ros::State> state_buffer_;
        boost::circular_buffer<iESEKF::IMUmeas> imu_buffer_;

        // Parameters
        std::string world_frame_;
        std::string body_frame_;
        bool publish_tf_;

        std::string imu_topic_;
        std::string gps_topic_;
        std::string wheel_odom_topic_;
        std::string pose_topic_;
        std::string mag_topic_;
        std::string baro_topic_;

        // Noise parameters
        double gyro_noise_;
        double accel_noise_;
        double gyro_bias_noise_;
        double accel_bias_noise_;
        double gps_noise_;

        // IMU tracking
        double last_imu_stamp_;

        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_odom_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
        rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;

        // Publishers (lifecycle-aware)
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> state_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>> pose_pub_;

        // TF
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

};

} // namespace ins_ros
