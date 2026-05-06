#pragma once

#include "gilda_lio/map.hpp"
#include "gilda_lio/ekf.hpp"

namespace gilda_lio {

class OdometryCore {
public:
    explicit OdometryCore();
    ~OdometryCore() = default;

    // Singleton
    static OdometryCore& getInstance(){
        static OdometryCore* loc = new OdometryCore();
        return *loc;
    }

    // Initialize/Load config
    void initialize(const gilda_lio::Config& config);

    // Process IMU measurement (propagate state)
    void processIMU(lie_odyssey::IMUmeas& imu);

    // Process LiDAR scan (update state with plane constraints)
    void processScan(const pcl::PointCloud<LioPointType>::Ptr& scan, double stamp);

    // Compute point-to-plane measurement model
    void pointToPlaneResidual(const iESEKF::Group& X_now, iESEKF::Measurement& z, iESEKF::HMat& H);

    // Get current state (baselink) in world frame
    State getState() const { return state_; }

    // Get current state (lidar) in world frame 
    State getLiDARState() const { return state_; } // To-Do: take into account extrinsics

    // Get last processed scan transformed to world frame
    pcl::PointCloud<LioPointType>::Ptr getWorldScan() const { return world_scan_; }

    // Get raw processed scan transformed to world frame (not downsampled)
    pcl::PointCloud<LioPointType>::Ptr getRawWorldScan() const { return raw_world_scan_; }

    // Get deskewed pointcloud
    pcl::PointCloud<LioPointType>::ConstPtr getDeskewedScan() const { return deskewed_scan_; }

    // Get pointcloud to match
    pcl::PointCloud<LioPointType>::Ptr getScanToMatch() const { return pc2match_; }

    // Check if initialized and calibrated
    bool isCalibrated() const { return calibrated_; }

    // Get sensor type
    gilda_lio::SensorType get_sensor_type() const;

    // Get covariance (6x6 pose + 6x6 twist flattened row-major)
    std::vector<double> getPoseCovariance() const;
    std::vector<double> getTwistCovariance() const;

    // Get map backend 
    const Map::MapType* getMap() const;

private:
    void initFilter();
    void initState();
    void imuToBody(lie_odyssey::IMUmeas& imu);
    void propagateIMU(const lie_odyssey::IMUmeas& imu);
    std::vector<State> integrateImu(double start_time, double end_time);
    bool propagatedFromTimeRange(double start_time, double end_time,
                                boost::circular_buffer<State>::reverse_iterator& begin_prop_it,
                                boost::circular_buffer<State>::reverse_iterator& end_prop_it);
    pcl::PointCloud<LioPointType>::Ptr deskewScan(pcl::PointCloud<LioPointType>::Ptr& pc, double& start_time);

    // Disable copy/move functionality
    OdometryCore(const OdometryCore&) = delete;
    OdometryCore& operator=(const OdometryCore&) = delete;
    OdometryCore(OdometryCore&&) = delete;
    OdometryCore& operator=(OdometryCore&&) = delete;

    Config config_;
    State state_;

    // EKF filter
    std::unique_ptr<iESEKF::Filter> filter_;
    std::mutex mtx_filter;

    // Buffers
    boost::circular_buffer<lie_odyssey::IMUmeas> imu_buffer_;
    boost::circular_buffer<State> state_buffer_;
    std::condition_variable cv_prop_stamp;
    std::mutex mtx_state_buf; // mutex for avoiding multiple thread access to the buffer

    // Filters
    pcl::CropBox<LioPointType> crop_filter_;
    pcl::VoxelGrid<LioPointType> voxel_filter_;

    // Pointclouds
    pcl::PointCloud<LioPointType>::ConstPtr deskewed_scan_;
    pcl::PointCloud<LioPointType>::Ptr world_scan_;
    pcl::PointCloud<LioPointType>::Ptr raw_world_scan_;
    pcl::PointCloud<LioPointType>::Ptr pc2match_;

    // Map object
    std::unique_ptr<Map> map_;

    // Timing
    double scan_stamp_ = 0.0;
    double prev_scan_stamp_ = 0.0;
    double last_imu_stamp_ = 0.0;
    double first_imu_stamp_ = 0.0;
    double last_propagate_stamp_ = -1.0;

    bool calibrated_ = false;
    int max_num_threads_;
};

} // namespace gilda_lio