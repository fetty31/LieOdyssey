#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "ins_ros/measurements/stamped_types.hpp"
#include "ins_ros/utils/trajectory_aligner.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ins_ros::utils::debug {

inline void publish_gps_fix(const Eigen::Vector3d& position,
                            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
                            std::vector<geometry_msgs::msg::Point>& gps_points_accumulated,
                            const std::string& frame_id,
                            const rclcpp::Time& now)
{
    geometry_msgs::msg::Point p;
    p.x = position.x();
    p.y = position.y();
    p.z = position.z();
    gps_points_accumulated.push_back(p);

    visualization_msgs::msg::Marker marker;

    marker.header.stamp = now;
    marker.header.frame_id = frame_id;

    marker.ns = "ins_ros_gps_debug";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;

    marker.color.a = 1.0;
    marker.color.b = 1.0;

    marker.points = gps_points_accumulated;

    publisher->publish(marker);
}

inline void publish_gps_odom(const measurements::StampedGps& stamped_gps,
                             const Eigen::Quaterniond& q,
                             rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher,
                             const std::string& world_frame,
                             const std::string& body_frame,
                             const rclcpp::Time& now)
{
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = world_frame;
    odom_msg.child_frame_id = body_frame;

    // Recover body origin position from GPS antenna position.
    // position_enu: GPS antenna position in ENU
    // lever_arm: GPS antenna position w.r.t. body, expressed in body frame.
    auto R = q.toRotationMatrix();
    auto gps_body = stamped_gps.meas.position_enu - R * stamped_gps.meas.lever_arm;

    odom_msg.pose.pose.position.x = gps_body.x();
    odom_msg.pose.pose.position.y = gps_body.y();
    odom_msg.pose.pose.position.z = gps_body.z();

    // Orientation is not observable from GPS, but we can still publish the current filter orientation for visualization.
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    // GPS position covariance.
    // ROS Odometry pose covariance layout:
    // [x, y, z, roll, pitch, yaw]
    //
    // R_gps is assumed to be the 3x3 ENU position covariance.
    odom_msg.pose.covariance.fill(0.0);

    odom_msg.pose.covariance[0]  = stamped_gps.R(0, 0);  // x-x
    odom_msg.pose.covariance[1]  = stamped_gps.R(0, 1);  // x-y
    odom_msg.pose.covariance[2]  = stamped_gps.R(0, 2);  // x-z

    odom_msg.pose.covariance[6]  = stamped_gps.R(1, 0);  // y-x
    odom_msg.pose.covariance[7]  = stamped_gps.R(1, 1);  // y-y
    odom_msg.pose.covariance[8]  = stamped_gps.R(1, 2);  // y-z

    odom_msg.pose.covariance[12] = stamped_gps.R(2, 0);  // z-x
    odom_msg.pose.covariance[13] = stamped_gps.R(2, 1);  // z-y
    odom_msg.pose.covariance[14] = stamped_gps.R(2, 2);  // z-z

    // No orientation information is provided by GPS.
    // We leave roll/pitch/yaw covariance at zero because 
    // this message is strictly for debugging. 

    // No velocity measurement from this GPS message.
    odom_msg.twist.covariance.fill(0.0);

    // Publish
    publisher->publish(odom_msg);
}

inline void publish_yaw(double yaw, 
                        const Eigen::Vector3d& position,
                        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher,
                        const std::string& frame_id,
                        const rclcpp::Time& now)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = now;
    marker.ns = "ins_ros_yaw_debug";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    marker.pose.orientation = tf2::toMsg(q);
    marker.scale.x = 1.0;
    marker.scale.y = 0.06;
    marker.scale.z = 0.06;
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
    publisher->publish(marker);
}

inline void publish_aligned_trajectories(const Eigen::Isometry3d& T,
                                         const TrajectoryAligner& trajectory_aligner,
                                         rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr source_pub,
                                         rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr target_pub,
                                         rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr source_aligned_pub,
                                         const std::string& frame_id,
                                         const rclcpp::Time& now)
{   
    auto sync_trajs = trajectory_aligner.synchronize();

    nav_msgs::msg::Path path_source;
    path_source.header.frame_id = frame_id;
    path_source.header.stamp = now;

    nav_msgs::msg::Path path_aligned;
    path_aligned.header.frame_id = frame_id;
    path_aligned.header.stamp = now;

    for(auto& pose : sync_trajs.source)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id;
        ps.header.stamp = now;
        ps.pose.position.x = pose.position.x();
        ps.pose.position.y = pose.position.y();
        ps.pose.position.z = pose.position.z();
        ps.pose.orientation.w = 1.0;
        path_source.poses.push_back(ps);

        auto pose_aligned = T * pose.position;
        ps.pose.position.x = pose_aligned.x();
        ps.pose.position.y = pose_aligned.y();
        ps.pose.position.z = pose_aligned.z();
        path_aligned.poses.push_back(ps);
    }

    nav_msgs::msg::Path path_target;
    path_target.header.frame_id = frame_id ;
    path_target.header.stamp = now;

    for(auto& pose : sync_trajs.target)
    {
        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = frame_id;
        ps.header.stamp = now;
        ps.pose.position.x = pose.position.x();
        ps.pose.position.y = pose.position.y();
        ps.pose.position.z = pose.position.z();
        ps.pose.orientation.w = 1.0;

        path_target.poses.push_back(ps);
    }

    source_pub->publish(path_source);
    target_pub->publish(path_target);
    source_aligned_pub->publish(path_aligned);
}



} // namespace ins_ros::utils::debug_publish