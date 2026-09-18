#pragma once

// Project includes
#include "ins_ros/state.hpp"
#include "ins_ros/ekf.hpp"

#include "ins_ros/geo/enu_converter.hpp"

#include "ins_ros/init/imu_orientation_initializer.hpp"
#include "ins_ros/init/gps_orientation_initializer.hpp"

#include "ins_ros/utils/frame_transform.hpp"

#include "ins_ros/measurements/measurement_handler.hpp"

#include "ins_ros/sensors/baro_handler.hpp"
#include "ins_ros/sensors/gps_handler.hpp"
#include "ins_ros/sensors/mag_handler.hpp"
#include "ins_ros/sensors/odom_handler.hpp"
#include "ins_ros/sensors/relative_odom_handler.hpp"
#include "ins_ros/sensors/wheel_handler.hpp"
#include "ins_ros/sensors/yaw_handler.hpp"

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

    // debug/visualization
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// TF
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

// Utilities
#include <memory>
#include <optional>
#include <string>
#include <vector>
#include <array>

#include <Eigen/Geometry>

namespace ins_ros {

class INSEstimator : public rclcpp_lifecycle::LifecycleNode
{

    // TYPES

    public:

        using CallbackReturn =
            rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

        using ROSCovariance = std::array<double, 36>;

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

        // --- Thin ROS callbacks: only convert + push into MeasurementHandler ---
        void imu_callback(const sensor_msgs::msg::Imu& msg);
        void gps_callback(const sensor_msgs::msg::NavSatFix& msg);
        void wheel_odom_callback(const geometry_msgs::msg::TwistStamped& msg);
        void odom_callback(const nav_msgs::msg::Odometry& msg);
        void mag_callback(const sensor_msgs::msg::MagneticField& msg);
        void baro_callback(const sensor_msgs::msg::FluidPressure& msg);

        // --- Fixed-frequency estimation loop ---
        void estimation_timer_callback();

        void processMeasurement(const measurements::StampedGps& gps);
        void processMeasurement(const measurements::StampedOdom& odom);
        void processMeasurement(const measurements::StampedWheel& wheel);
        void processMeasurement(const measurements::StampedMag& mag);
        void processMeasurement(const measurements::StampedBaro& baro);
        void processMeasurement(const measurements::StampedYaw& yaw);

        void process_gps(const measurements::StampedGps& gps);
        void process_odom(const measurements::StampedOdom& odom);
        void process_wheel(const measurements::StampedWheel& wheel);
        void process_mag(const measurements::StampedMag& mag);
        void process_baro(const measurements::StampedBaro& baro);

        // void process_relative_odom_at(double filter_time);

        void refresh_state_from_filter(double stamp);

        // GPS latency handling: rewind to snapshot, update, re-propagate IMU.
        // bool apply_gps_with_rewind(const measurements::StampedGps& gps, double filter_time);
        void apply_gps_direct(const measurements::StampedGps& gps);

        bool try_initialize_orientation();

        void setState();

        // --- Setup helpers (called during configure) ---
        void declare_parameters();
        void load_parameters();
        void setup_subscriptions();
        void setup_publishers();
        void setup_timer();

        // --- ROS <-> Library conversion helpers ---
        double stampToSec(const rclcpp::Time& header_stamp);
        void initializeTime(double t);
        void handleTimeReset(double t);
        void handleTimeJump(double t, double dt);

        void from_ros_to_ins(const sensor_msgs::msg::Imu& in, iESEKF::IMUmeas& out);
        void from_ros_to_ins(const geometry_msgs::msg::PoseStamped& in, ins_ros::State& out);
        void from_ros_to_ins(const nav_msgs::msg::Odometry& in, ins_ros::State& out);

        void from_ins_to_ros(const ins_ros::State& in, nav_msgs::msg::Odometry& out,
            const std::optional<ROSCovariance>& pose_cov = std::nullopt,
            const std::optional<ROSCovariance>& twist_cov = std::nullopt);
        void from_ins_to_ros(const ins_ros::State& in, geometry_msgs::msg::PoseWithCovarianceStamped& out, 
            const std::optional<ROSCovariance>& pose_cov = std::nullopt);

        // Additional helpers
        void publish_odom();
        void publish_pose();
        void broadcast_tf(const ins_ros::State& in, bool now = true);
        bool transform_imu_to_base_link(const sensor_msgs::msg::Imu& msg, iESEKF::IMUmeas& imu);
        void initialize_orientation();

        void print_state(const std::string& prefix, const ins_ros::State& state);

        void publish_gps_debug(const Eigen::Vector3d& gps_position);
        void publish_yaw_debug(double yaw, const Eigen::Vector3d& position_enu);

    // VARIABLES

    private:

        // EKF
        iESEKF::Filter filter_;
        int max_iters_;
        double tolerance_;

        // Centralized measurement buffers + state history for rewind.
        measurements::MeasurementHandler meas_handler_;

        // Filter time base: ROS sensor stamp (seconds) of last processed IMU.
        double filter_time_{-1.0};
        double filter_init_time_{-1.0};
        bool filter_time_initialized_{false};

        // State (mirrors filter after each timer tick; read by callbacks for bias/debug).
        ins_ros::State state_;

        // TF
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        tf2_ros::Buffer tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        // Extrinsics
        utils::FrameTransform imu_to_base_;
        utils::FrameTransform lio_to_base_;
        utils::FrameTransform initial_enu_base_;

            // Complete transform that maps LIO/VIO poses
            // (LIO world + LIO body) into the EKF world/body representation (it includes lio_to_base_).
        utils::FrameTransform lio_to_enu_; 

        // IMU
        std::string imu_topic_{""};
        bool estimate_imu_bias_{false};
        bool estimate_imu_orientation_{false};
        double last_imu_stamp_{-1.0};
        State::V3 previous_omega_base_{State::V3::Zero()};

        // Orientation initializers
        std::unique_ptr<init::IMUOrientationInitializer> imu_orientation_initializer_;
        std::unique_ptr<init::GPSOrientationInitializer> gps_orientation_initializer_;
        std::unique_ptr<init::GPSOrientationInitializer> gps_orientation_continuous_;
        bool orientation_initialized_{false};

        // GPS / ENU converter
        std::string gps_topic_{""};
        ENUConverter enu_converter_;
        bool trust_gps_covariance_{false};
        State::V3 gps_lever_arm_{State::V3::Zero()};
        State::V3 gps_noise_{State::V3::Zero()};

        // 3D Odometry
        std::string odom_topic_{""};
        bool trust_odom_pose_covariance_{false};
        bool trust_odom_velocity_covariance_{false};
        State::V3 odom_position_noise_{State::V3::Zero()};
        State::V3 odom_orientation_noise_{State::V3::Zero()};
        State::V3 odom_velocity_noise_{State::V3::Zero()};

        std::optional<iESEKF::relative_odom::RelativeOdomMeasurement> odom_reference_;

        // Wheel odom
        std::string wheel_odom_topic_{""};
        State::V3 wheel_odom_noise_{State::V3::Zero()};

        // Magnetometer
        std::string mag_topic_{""};

        // Barometer
        std::string baro_topic_{""};

        // Parameters
        std::string world_frame_;
        std::string body_frame_;
        bool publish_tf_;

        // When true, stamp all incoming measurements with the local receive
        // time instead of trusting msg.header.stamp (for sensors whose clock
        // is not synchronized with the PC clock).
        bool use_receive_stamp_{false};

        // Estimation loop / synchronization parameters
        double estimation_rate_{100.0};
        double history_window_s_{5.0};
        double gps_rewind_threshold_{0.05};
        double gps_max_age_{2.0};
        double sync_tolerance_odom_{0.05};
        double sync_tolerance_wheel_{0.05};
        double sync_tolerance_mag_{0.05};
        double sync_tolerance_baro_{0.05};
        double sync_tolerance_yaw_{0.10};
        double sync_future_tolerance_{0.02};
        std::size_t imu_buffer_capacity_{2000};
        std::size_t measurement_capacity_{1000};

        // Process noise parameters
        double gyro_noise_;
        double accel_noise_;
        double gyro_bias_noise_;
        double accel_bias_noise_;
        
        // Subscribers
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr       imu_sub_;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr wheel_odom_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr mag_sub_;
        rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr baro_sub_;

        // Fixed-frequency estimation timer
        rclcpp::TimerBase::SharedPtr estimation_timer_;

        // Publishers (lifecycle-aware)
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>> state_pub_;
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<
            geometry_msgs::msg::PoseWithCovarianceStamped>> pose_pub_;

        // Publishers (debug/visualization)
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_gps_pub_;
        std::vector<geometry_msgs::msg::Point> debug_gps_points_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr debug_odom_pub_;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr debug_yaw_pub_;
        
};

} // namespace ins_ros
