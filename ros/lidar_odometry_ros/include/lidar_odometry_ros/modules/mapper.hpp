#ifndef __LIDAR_ODOMETRY_ROS_MAPPER_HPP__
#define __LIDAR_ODOMETRY_ROS_MAPPER_HPP__

#include "lidar_odometry_ros/common.hpp"
#include "lidar_odometry_ros/objects/octree.hpp"
#include "lidar_odometry_ros/objects/state.hpp"
#include "lidar_odometry_ros/objects/match.hpp"
#include "lidar_odometry_ros/utils/config.hpp"

using namespace lidar_odometry_ros;

class lidar_odometry_ros::Mapper {

    // Variables

    private:
        std::unique_ptr<octree::Octree> map_;

        Config::Mapping config;

        double last_map_time;

        int num_threads_;

    public:
        Matches matches;

    // Methods

    public:
        Mapper();

        void set_num_threads(int n);
        void set_config(const Config::Mapping& cfg);

        bool exists();
        int size();
        double last_time();

        Matches match(State, pcl::PointCloud<PointType>::Ptr&);

        void add(pcl::PointCloud<PointType>::Ptr&, double time);

    private:
        Match match_plane(Eigen::Vector4f& p, Eigen::Vector4f& p_local);

    // Singleton 

    public:
        static Mapper& getInstance() {
            static Mapper* mapper = new Mapper();
            return *mapper;
        }

    private:
        // Disable copy/move capabilities
        Mapper(const Mapper&) = delete;
        Mapper(Mapper&&) = delete;

        Mapper& operator=(const Mapper&) = delete;
        Mapper& operator=(Mapper&&) = delete;

};

#endif
