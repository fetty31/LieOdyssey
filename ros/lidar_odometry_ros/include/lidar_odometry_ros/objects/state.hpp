#ifndef __LIDAR_ODOMETRY_ROS_STATE_HPP__
#define __LIDAR_ODOMETRY_ROS_STATE_HPP__

#include "lidar_odometry_ros/common.hpp"

#include "lidar_odometry_ros/modules/filters/iesekf_sgal3.hpp"
// #include "lidar_odometry_ros/modules/filters/iesekf_se23.hpp"
// #include "lidar_odometry_ros/modules/filters/iesekf_so3.hpp"
// #include "lidar_odometry_ros/modules/filters/iesekf_se3.hpp"

class lidar_odometry_ros::State{

    public:

        struct IMUbias;

        Eigen::Vector3f p;      // position in global/world frame
        Eigen::Quaternionf q;   // orientation in global/world frame
        Eigen::Vector3f v;      // linear velocity
        Eigen::Vector3f g;      // gravity vector
        
        Eigen::Vector3f w;      // angular velocity (IMU input)
        Eigen::Vector3f a;      // linear acceleration (IMU input)

        double time;

        struct IMUbias {
            Eigen::Vector3f gyro;
            Eigen::Vector3f accel;
        } b;                    // IMU bias in base_link/body frame 

        State();
        State(const iESEKF::Group& s);
        State(const iESEKF::Group& s, double t);
        State(const iESEKF::Group& s, double t, Eigen::Vector3f a, Eigen::Vector3f w);
        State(Eigen::Matrix4f& s);

        void operator+=(const State& s);

        void update(double t);

        Eigen::Matrix4f get_RT();           // get Rotation & Translation matrix

        Eigen::Matrix4f get_RT_inv();       // get inverted Rotation & Translation matrix
};

#endif