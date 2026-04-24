#include "lidar_odometry_ros/objects/match.hpp"

// class lidar_odometry_ros::Match
    // public

        lidar_odometry_ros::Match::Match(const Eigen::Vector3f& p_global, 
                                const Eigen::Vector3f& p_local, 
                                const lidar_odometry_ros::Plane& H) : p_global(p_global), p_local(p_local), plane(H)
        {
            this->dist = this->plane.dist2plane(p_global);
        }

        bool lidar_odometry_ros::Match::lisanAlGaib(){
            return this->plane.good_fit();
        }

        void lidar_odometry_ros::Match::update_global(lidar_odometry_ros::State& s){
            Eigen::Vector4f p4_global = s.get_RT() * this->get_4Dlocal();
            this->p_global = p4_global.head(3);
        }

        Eigen::Vector4f lidar_odometry_ros::Match::get_4Dglobal(){
            return Eigen::Vector4f(this->p_global(0), this->p_global(1), this->p_global(2), 1.0);
        }

        Eigen::Vector4f lidar_odometry_ros::Match::get_4Dlocal(){
            return Eigen::Vector4f(this->p_local(0), this->p_local(1), this->p_local(2), 1.0);
        }

        Eigen::Vector3f lidar_odometry_ros::Match::get_global_point(){
            return this->p_global;
        }

        Eigen::Vector3f lidar_odometry_ros::Match::get_local_point(){
            return this->p_local;
        }
