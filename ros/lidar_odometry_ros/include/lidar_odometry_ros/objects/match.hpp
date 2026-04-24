#ifndef __LIDAR_ODOMETRY_ROS_MATCH_HPP__
#define __LIDAR_ODOMETRY_ROS_MATCH_HPP__

#include "lidar_odometry_ros/common.hpp"
#include "lidar_odometry_ros/objects/state.hpp"
#include "lidar_odometry_ros/objects/plane.hpp"

class lidar_odometry_ros::Match{

    private:
        Eigen::Vector3f p_global;
        Eigen::Vector3f p_local;

    public:
        lidar_odometry_ros::Plane plane;
        float dist;

        Match(const Eigen::Vector3f& p_global, 
            const Eigen::Vector3f& p_local, 
            const lidar_odometry_ros::Plane& H);
        Match() = default;

        bool lisanAlGaib(); // whether is the chosen one :)

        void update_global(lidar_odometry_ros::State& s);

        Eigen::Vector4f get_4Dglobal();
        Eigen::Vector4f get_4Dlocal();
        Eigen::Vector3f get_global_point();
        Eigen::Vector3f get_local_point();
};

#endif