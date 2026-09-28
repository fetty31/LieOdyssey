#pragma once

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <iomanip> // setprecision()
#include <condition_variable>
#include <boost/circular_buffer.hpp>
#include <boost/range/adaptor/indexed.hpp>
#include <boost/range/adaptor/adjacent_filtered.hpp>
#include <boost/range/adaptor/filtered.hpp>

#define PCL_NO_PRECOMPILE
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_config.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>

namespace gilda_lio {

enum class SensorType { OUSTER, VELODYNE, HESAI, LIVOX, UNKNOWN };

struct Point {
    Point(): data{0.f, 0.f, 0.f, 1.f} {}
    Point(float x, float y, float z): data{x, y, z, 1.f} {}

    PCL_ADD_POINT4D;
    float intensity;
    union {
        std::uint32_t t;   // (Ouster) time since beginning of scan in nanoseconds
        float time;        // (Velodyne) time since beginning of scan in seconds
        double timestamp;  // (Hesai) absolute timestamp in seconds
                           // (Livox) absolute timestamp in (seconds * 10e9)
    };
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

#if PCL_VERSION_COMPARE(<, 1, 11, 0)
    #include <boost/make_shared.hpp>
	// Use Boost for older versions of PCL
	template <typename T>
	using shared_ptr = boost::shared_ptr<T>;

	template <typename T, typename... Args>
	boost::shared_ptr<T> make_shared(Args&&... args) {
    	return boost::make_shared<T>(std::forward<Args>(args)...);
	}
#else
	// Use std::shared_ptr for PCL >= 1.11.0
	template <typename T>
	using shared_ptr = std::shared_ptr<T>;

	template <typename T, typename... Args>
	std::shared_ptr<T> make_shared(Args&&... args) {
    	return std::make_shared<T>(std::forward<Args>(args)...);
	}
#endif

struct Config {
    // Topics
    std::string lidar_topic = "/points_raw";
    std::string imu_topic = "/imu";

    // Frames
    std::string world_frame = "map";
    std::string body_frame = "body";

    // General
    int num_threads = 1;
    SensorType sensor_type = SensorType::HESAI;
    bool publish_tf = true;
    bool motion_compensation = true; // whether to deskew input pointcloud
    bool end_of_sweep = false;       // whether to take last point stamp as reference time (pointcloud preprocessing)
    bool debug = true;               // whether to save intermediate pointclouds for visualization/debugging purposes
    bool time_offset = false;        // whether to sync imu-lidar (not accurate approach)

    // IMU extrinsics (IMU -> body)
    Eigen::Isometry3f imu_extr = Eigen::Isometry3f::Identity();

    // LiDAR extrinsics (LiDAR -> body)
    Eigen::Isometry3f lidar_extr = Eigen::Isometry3f::Identity();

    // IMU intrinsics
    Eigen::Vector3f accel_bias{0.f, 0.f, 0.f};
    Eigen::Vector3f gyro_bias{0.f, 0.f, 0.f};
    Eigen::Matrix3f imu_axis_matrix{Eigen::Matrix3f::Identity()};
    /*(if your IMU doesn't comply with axis system ISO-8855, 
    this matrix is meant to map its current orientation with respect to the standard axis system)
        Y-pitch
        ^   
        |  
        | 
        |
    Z-yaw o-----------> X-roll
    */

    // Calibration flags
    bool gravity_align = false;
    bool calibrate_accel = false;
    bool calibrate_gyro = false;
    double imu_calib_time = 3.0;

    // Filters
    bool crop_active = false;
    Eigen::Vector3f crop_min{-2.f, -2.f, -2.f};
    Eigen::Vector3f crop_max{2.f, 2.f, 2.f};

    bool voxel_active = true;
    Eigen::Vector3f voxel_size{0.5f, 0.5f, 0.5f};

    bool dist_active = false;
    float min_dist = 0.0f;

    bool rate_active = false;
    int rate_value = 2;

    // EKF parameters
    int max_ekf_iters = 10;
    double ekf_tolerance = 0.001;
    double cov_gyro = 0.01;
    double cov_acc = 0.1;
    double cov_bias_gyro = 0.001;
    double cov_bias_acc = 0.01;

    // Mapping parameters
    int max_num_matches = 5000;
    int max_pc2match = 10000;
    int n_points_match = 5;
    double max_dist_plane = 2.0;
    double resolution = 0.5;
    double plane_threshold = 0.1;
    double chi_square_threshold = 7.81;
    int update_threshold = 5;
    int max_buffer_size = 50;
    double noise_floor = 0.01;
    double lidar_noise = 0.01;
    double lidar_angle_noise = 0.001;
    double lidar_range_noise = 0.00001;
};

template <typename T>
std::string to_string_with_precision(const T a_value, const int n = 6)
{
    std::ostringstream out;
    out.precision(n);
    out << std::fixed << a_value;
    return out.str();
}

inline const char* to_string(SensorType s) {
    switch (s) {
        case SensorType::OUSTER:   return "OUSTER";
        case SensorType::VELODYNE: return "VELODYNE";
        case SensorType::HESAI:    return "HESAI";
        case SensorType::LIVOX:    return "LIVOX";
        default:                   return "UNKNOWN";
    }
}

template <typename Array>
inline int binary_search_tailored(const Array& sorted_v, double t) {
    int high, mid, low;
    low = 0; high = sorted_v.size()-1;
    
    while(high >= low){
        mid = (low + high)/2;
        (sorted_v[mid].time > t) ? high = mid - 1 : low = mid + 1;
    }

    // Return the leftest value (older time stamp)
    if(high < 0) return 0;
    return high;
}

} // namespace gilda_lio

POINT_CLOUD_REGISTER_POINT_STRUCT(gilda_lio::Point,
                                 (float, x, x)
                                 (float, y, y)
                                 (float, z, z)
                                 (float, intensity, intensity)
                                 (std::uint32_t, t, t)
                                 (float, time, time)
                                 (double, timestamp, timestamp))

typedef gilda_lio::Point LioPointType;
typedef pcl::PointXYZ MapPoint;
typedef std::vector<MapPoint> MapPoints;