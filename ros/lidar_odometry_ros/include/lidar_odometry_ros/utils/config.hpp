#ifndef __LIDAR_ODOMETRY_ROS_CONFIG_HPP__
#define __LIDAR_ODOMETRY_ROS_CONFIG_HPP__

#include "lidar_odometry_ros/common.hpp"

struct lidar_odometry_ros::Config{

    struct Topics{
        std::string lidar;
        std::string imu;
    } topics;

    struct Extrinsics{
        std::vector<float> imu2baselink_t;
        std::vector<float> imu2baselink_R;
        std::vector<float> lidar2baselink_t;
        std::vector<float> lidar2baselink_R;
    } extrinsics;

    struct Intrinsics{
        std::vector<float> accel_bias;
        std::vector<float> gyro_bias;
        std::vector<float> imu_sm;
    } intrinsics;

    struct Filters{
        std::vector<float> cropBoxMin;  // crop filter
        std::vector<float> cropBoxMax;  // crop filter
        bool crop_active;               // crop filter
        std::vector<float> leafSize;    // voxel grid filter
        bool voxel_active;              // voxel grid filter
        double min_dist;                // norm/dist filter
        bool dist_active;               // norm/dist filter
        int rate_value;                 // time rate filter
        bool rate_active;               // time rate filter
        float fov_angle;                // FoV filter
        bool fov_active;                // FoV filter
    } filters;

    struct iESEKFcfg{
        int MAX_NUM_ITERS;          // max num of iterations of the extended KF
        double TOLERANCE;           // ESEKF tolerance
        double cov_gyro;            // covariance ang. velocity
        double cov_acc;             // covariance lin. accel.
        double cov_bias_gyro;       // covariance bias ang. vel.
        double cov_bias_acc;        // covariance bias lin. accel.
    } ekf;

    struct Mapping{
        int NUM_MATCH_POINTS;   // minimum num of points that constitute a match
        int MAX_NUM_MATCHES;    // max num of matches (helps to reduce comp. load)
        int MAX_NUM_PC2MATCH;   // max num of points to match (helps to reduce comp. load)
        double MAX_DIST_PLANE;  // max distance between points to be considered a plane
        double LIDAR_NOISE;     // initial measurement noise from LiDAR sensor
        double PLANE_THRESHOLD; // threshold to consider an estimated plane is actually a plane (also used for deciding if point belongs to plane )
        struct Octree{
            int bucket_size;    //  maximum number of points/gaussians allowed in an octant before it gets subdivided
            double min_extent;   //  minimum extent of the octant (used to stop subdividing)
        } octree;
    } mapping;

    // Flags
    bool gravity_align;         // whether to estimate gravity vector
    bool calibrate_accel;       // whether to estimate linear accel. bias
    bool calibrate_gyro;        // whether to estimate ang. velocity bias
    bool time_offset;           // whether to take into account the time offset
    bool end_of_sweep;          // whether the sweep reference time is w.r.t. the start or the end of the scan (only applies to VELODYNE/OUSTER)
    bool motion_compensation;   // whether to perform motion compensation (deskewing)

    bool debug;         // whether to copy intermediate point clouds into aux variables (for visualization)
    bool verbose;       // whether to print debugging/performance board

    // Other
    int sensor_type;        // LiDAR type
    int num_threads;        // num of threads to be used by OpenMP
    double imu_calib_time;  // time to be estimating IMU biases (assuming stand-still)

};

#endif
