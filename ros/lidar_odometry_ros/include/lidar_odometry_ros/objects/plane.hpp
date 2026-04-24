#ifndef __LIDAR_ODOMETRY_ROS_PLANE_HPP__
#define __LIDAR_ODOMETRY_ROS_PLANE_HPP__

#include "lidar_odometry_ros/common.hpp"
#include "lidar_odometry_ros/utils/config.hpp"

class lidar_odometry_ros::Plane{

    public:

        Plane(const MapPoints& p, const std::vector<float>& d, 
              Config::Mapping* config_ptr);
        Plane() = default;

        Eigen::Vector4f get_normal();
        bool good_fit();

        float dist2plane(const Eigen::Vector3f&) const;
        float dist2plane(const PointType&) const;

        bool on_plane(const Eigen::Vector3f&);
        bool on_plane(const PointType&);
        
        bool enough_points(const MapPoints& p);
        bool close_enough(const std::vector<float>& d);

    private:
        Eigen::Vector3f centroid;
        Eigen::Vector4f n_ABCD; // plane normal vector
        bool is_plane;

        Config::Mapping* cfg_ptr;

        void fit_plane(const MapPoints&);

        Eigen::Vector4f estimate_plane(const MapPoints&);

        bool plane_eval(const Eigen::Vector4f&, const MapPoints&, const float&);

        Eigen::Vector3f get_centroid(const MapPoints&);

};

#endif
