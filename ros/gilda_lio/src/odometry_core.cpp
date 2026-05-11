#include "gilda_lio/odometry_core.hpp"

gilda_lio::OdometryCore::OdometryCore() 
{
    this->deskewed_scan_  = pcl::PointCloud<LioPointType>::ConstPtr (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>());
    this->world_scan_     = pcl::PointCloud<LioPointType>::Ptr (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>());
    this->raw_world_scan_ = pcl::PointCloud<LioPointType>::Ptr (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>());
    this->pc2match_       = pcl::PointCloud<LioPointType>::Ptr (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>());
}

void gilda_lio::OdometryCore::initialize(const gilda_lio::Config& config)
{
    this->config_ = config;

    // Set num of threads
    this->max_num_threads_ = omp_get_max_threads();
    if(max_num_threads_ > config.num_threads) this->max_num_threads_ = config.num_threads;

    // Mapping set up
    Map::Config map_cfg;
    map_cfg.min_plane_points = config.n_points_match;
    map_cfg.max_dist_thresh = static_cast<Map::Scalar>(config.max_dist_plane);
    map_cfg.max_num_pc2match = static_cast<std::size_t>(config.max_pc2match);
    map_cfg.resolution = static_cast<Map::Scalar>(config.resolution);
    map_cfg.update_thresh = static_cast<std::size_t>(config.update_threshold);
    map_cfg.max_buffer_size = static_cast<std::size_t>(config.max_buffer_size);
    map_cfg.planarity_thresh = static_cast<Map::Scalar>(config.plane_threshold);
    map_cfg.chi_square_thresh = static_cast<Map::Scalar>(config.chi_square_threshold);
    map_cfg.noise_floor = static_cast<Map::Scalar>(config.noise_floor);
    map_cfg.lidar_angle_noise = static_cast<Map::Scalar>(config.lidar_angle_noise);
    map_cfg.lidar_range_noise = static_cast<Map::Scalar>(config.lidar_range_noise);

    map_ = std::make_unique<Map>(max_num_threads_,
                                map_cfg
                                );

    // Initialize Iterated Error-State Kalman Filter on Manifolds
    this->initFilter();

    // Set buffer capacity
    this->imu_buffer_.set_capacity(2000);
    this->state_buffer_.set_capacity(2000);

    // PCL filters setup
    this->crop_filter_.setNegative(true);
    this->crop_filter_.setMin(Eigen::Vector4f(config.crop_min[0], config.crop_min[1], config.crop_min[2], 1.0));
    this->crop_filter_.setMax(Eigen::Vector4f(config.crop_max[0], config.crop_max[1], config.crop_max[2], 1.0));

    this->voxel_filter_.setLeafSize(config.voxel_size(0), config.voxel_size(1), config.voxel_size(2));

    // IMU intrinsics
    this->state_.bias.a = config.accel_bias;
    this->state_.bias.w = config.gyro_bias;

    // IMU extrinsics
    this->state_.p = config.imu_extr.translation();
    Eigen::Matrix3f R = config.imu_extr.rotation();
    Eigen::Quaternionf q(R); q.normalize();
    this->state_.q = q;

    // Avoid unnecessary warnings from PCL
    pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

    // Initial calibration
    if( !(config.gravity_align || config.calibrate_accel || config.calibrate_gyro) ){ // no need for automatic calibration
        this->initState();
        this->calibrated_ = true;
    }
}

// Process IMU measurement (propagate state)
void gilda_lio::OdometryCore::processIMU(lie_odyssey::IMUmeas& imu)
{
    GILDA_PROFILE_FUNCTION(profiler_);

    this->imuToBody(imu);

    if(this->first_imu_stamp_ == 0.0)
        this->first_imu_stamp_ = imu.stamp;

    // IMU calibration procedure - do only while the robot is in stand still!
    if (!this->calibrated_) {

        static int num_samples = 0;
        static Eigen::Vector3f gyro_avg (0., 0., 0.);
        static Eigen::Vector3f accel_avg (0., 0., 0.);
        static bool print = true;

        if ( (imu.stamp - this->first_imu_stamp_) < config_.imu_calib_time) {

            num_samples++;

            gyro_avg(0) += static_cast<float>(imu.gyro(0));
            gyro_avg(1) += static_cast<float>(imu.gyro(1));
            gyro_avg(2) += static_cast<float>(imu.gyro(2));

            accel_avg(0) += static_cast<float>(imu.accel(0));
            accel_avg(1) += static_cast<float>(imu.accel(1));
            accel_avg(2) += static_cast<float>(imu.accel(2));

            if(print) {
                std::cout << std::endl << " Calibrating IMU for " << config_.imu_calib_time << " seconds... \n";
                std::cout.flush();
                print = false;
            }

        } else {

            gyro_avg /= num_samples;
            accel_avg /= num_samples;

            if (config_.gravity_align) {

                std::cout << " Accel mean: " << "[ " << accel_avg[0] << ", " << accel_avg[1] << ", " << accel_avg[2] << " ]\n";

                // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
                Eigen::Vector3f grav_vec = (accel_avg - this->state_.bias.a).normalized() * 9.81f;
                if (grav_vec.norm() > 1e-6f) {
                    Eigen::Quaternionf grav_q =
                        Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0.f, 0.f, 9.81f));

                    std::cout << " Gravity mean: " << "[ " << grav_vec[0] << ", " << grav_vec[1] << ", " << grav_vec[2] << " ]\n";
                    
                    // set gravity aligned orientation
                    this->state_.q = grav_q.normalized();

                    // set estimated gravity vector
                    this->state_.g = grav_vec;
                }

            }

            if (config_.calibrate_accel) {

                // subtract gravity from avg accel to get bias
                this->state_.bias.a = accel_avg - this->state_.g;

                std::cout << " Accel biases [xyz]: " << to_string_with_precision(this->state_.bias.a[0], 8) << ", "
                                                    << to_string_with_precision(this->state_.bias.a[1], 8) << ", "
                                                    << to_string_with_precision(this->state_.bias.a[2], 8) << std::endl;
            }

            if (config_.calibrate_gyro) {

                this->state_.bias.w = gyro_avg;

                std::cout << " Gyro biases  [xyz]: " << to_string_with_precision(this->state_.bias.w[0], 8) << ", "
                                                    << to_string_with_precision(this->state_.bias.w[1], 8) << ", "
                                                    << to_string_with_precision(this->state_.bias.w[2], 8) << std::endl;
            }

            // Set calib flag
            this->calibrated_ = true;

            // Set initial KF state
            this->initState();

            // Initial attitude
            auto euler = this->state_.q.toRotationMatrix().eulerAngles(2, 1, 0);
            double yaw = euler[0] * (180.0/M_PI);
            double pitch = euler[1] * (180.0/M_PI);
            double roll = euler[2] * (180.0/M_PI);

            // use alternate representation if the yaw is smaller
            if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
                yaw   = remainder(yaw + 180.0,   360.0);
                pitch = remainder(180.0 - pitch, 360.0);
                roll  = remainder(roll + 180.0,  360.0);
            }
            std::cout << " Estimated initial attitude:" << std::endl;
            std::cout << "   Roll  [deg]: " << to_string_with_precision(roll, 4) << std::endl;
            std::cout << "   Pitch [deg]: " << to_string_with_precision(pitch, 4) << std::endl;
            std::cout << "   Yaw   [deg]: " << to_string_with_precision(yaw, 4) << std::endl;
            std::cout << std::endl;

        }

    } else {

        // Quick alignment w.r.t standard frames (yaw, pitch, roll), only for IMU not complying with ISO8855
        Eigen::Vector3d lin_accel_corrected = (config_.imu_axis_matrix.cast<double>() * imu.accel);
        Eigen::Vector3d ang_vel_corrected = (config_.imu_axis_matrix.cast<double>() * imu.gyro);

        imu.accel = lin_accel_corrected;
        imu.gyro  = ang_vel_corrected;
        imu.bias.accel = state_.bias.a.cast<double>();
        imu.bias.gyro  = state_.bias.w.cast<double>();

        // Store calibrated IMU measurements into imu buffer for manual integration later.
        this->imu_buffer_.push_front(imu);

        // iESEKF propagate state
        this->propagateIMU(imu);
        this->cv_prop_stamp.notify_one(); // Notify PointCloud thread that propagated IMU data exists for this time
    }

}

// Process LiDAR scan (update state with plane constraints)
void gilda_lio::OdometryCore::processScan(const pcl::PointCloud<LioPointType>::Ptr& scan, double stamp)
{
    GILDA_PROFILE_FUNCTION(profiler_);

    if(scan->points.size() < 1){
        std::cout << "gilda_lio::Input PointCloud is empty!\n";
        return;
    }

    if(!this->calibrated_)
        return;

    if(this->imu_buffer_.empty()){
        std::cout << "gilda_lio::IMU buffer is empty!\n";
        return;
    }

    pcl::PointCloud<LioPointType>::Ptr deskewed_Xt2_pc_ (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>());

{ // Preprocess scan (NaN removal, cropping, downsampling, motion compensation)
    GILDA_PROFILE_SCOPE(profiler_, "processScan/Preprocessing");

    // Remove NaNs
    std::vector<int> idx;
    scan->is_dense = false;
    pcl::removeNaNFromPointCloud(*scan, *scan, idx);

    // Crop Box Filter (1 m^2)
    if(this->config_.crop_active){
        this->crop_filter_.setInputCloud(scan);
        this->crop_filter_.filter(*scan);
    }

    // Distance & Time Rate filters
    static float min_dist = static_cast<float>(this->config_.min_dist);
    static int rate_value = this->config_.rate_value;
    std::function<bool(boost::range::index_value<LioPointType&, long>)> filter_f;
    
    if(this->config_.dist_active && this->config_.rate_active){
        filter_f = [this](boost::range::index_value<LioPointType&, long> p)
            { return (Eigen::Vector3f(p.value().x, p.value().y, p.value().z).norm() > min_dist)
                        && (p.index()%rate_value == 0); };
    }
    else if(this->config_.dist_active){
        filter_f = [this](boost::range::index_value<LioPointType&, long> p)
            { return (Eigen::Vector3f(p.value().x, p.value().y, p.value().z).norm() > min_dist); };
    }
    else if(this->config_.rate_active){
        filter_f = [this](boost::range::index_value<LioPointType&, long> p)
            { return (p.index()%rate_value == 0); };
    }else{
        filter_f = [this](boost::range::index_value<LioPointType&, long> p)
            { return true; };
    }

    pcl::PointCloud<LioPointType>::Ptr input_pc (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>());
    auto filtered_pc = scan->points 
                | boost::adaptors::indexed()
                | boost::adaptors::filtered(filter_f);
    for (auto it = filtered_pc.begin(); it != filtered_pc.end(); it++) {
        input_pc->points.push_back(it->value());
    }

    // Motion compensation
    deskewed_Xt2_pc_ = this->deskewScan(input_pc, stamp);
    /*NOTE: deskewed_Xt2_pc_ should be in base_link/body frame w.r.t last propagated state (Xt2) */

    // Voxel Grid Filter
    if (this->config_.voxel_active) { 
        pcl::PointCloud<LioPointType>::Ptr current_scan_
            (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>(*deskewed_Xt2_pc_));
        this->voxel_filter_.setInputCloud(current_scan_);
        this->voxel_filter_.filter(*current_scan_);
        this->pc2match_ = current_scan_;
    } else {
        this->pc2match_ = deskewed_Xt2_pc_;
    }

} // finish preprocessing

    if(this->pc2match_->points.size() > 1){

        // iESEKF observation stage
        this->mtx_filter.lock();

        // Update iESEKF measurements 
        {
        GILDA_PROFILE_SCOPE(profiler_, "processScan/iESEKF Update");
        this->filter_->update
                <iESEKF::Measurement, 
                iESEKF::HMat> (static_cast<iESEKF::Scalar>(config_.lidar_noise),
                                iESEKF::H_fun /*Measurement function*/);
        /*NOTE: update() will trigger the matching procedure
        in order to update the measurement stage of the KF with the computed point-to-plane distances*/
        }        


            // Get output state from iESEKF
        gilda_lio::State corrected_state;
        corrected_state.time = this->scan_stamp_;
        iESEKF::group_to_state(this->filter_->getState(), corrected_state);

        // Set estimated biases & gravity to constant
        if(this->config_.calibrate_gyro)  corrected_state.bias.w  = this->state_.bias.w;
        if(this->config_.calibrate_accel) corrected_state.bias.a  = this->state_.bias.a;
        if(this->config_.gravity_align)   corrected_state.g       = this->state_.g;

        // Update current state estimate
        auto last_imu = this->imu_buffer_.front();
        this->state_      = corrected_state;
        this->state_.w    = last_imu.gyro.cast<float>();
        this->state_.a    = last_imu.accel.cast<float>();

        this->mtx_filter.unlock();

        // Transform deskewed pc 
            // Get deskewed scan to add to map
        pcl::PointCloud<LioPointType>::Ptr mapped_scan (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>());
        mapped_scan->points.resize(pc2match_->points.size());

        // pcl::transformPointCloud (*this->pc2match, *mapped_scan, this->state_.get_transform()); // Not working for PCL 1.12
        #pragma omp parallel for num_threads(this->max_num_threads_)
        for(std::size_t i=0; i < pc2match_->points.size(); i++){
            LioPointType pt = pc2match_->points[i];
            pt.getVector3fMap() = this->state_.get_transform() * pt.getVector3fMap(); 
            mapped_scan->points[i] = pt;
            /*NOTE: pc2match must be in base_link frame w.r.t Xt2 frame for this transform to work.
                mapped_scan is in world/global frame.
            */
        }

            // Get final scan to output (in world/global frame)
        this->world_scan_ = mapped_scan; // mapped_scan = final_scan (for now)
        // pcl::transformPointCloud (*this->pc2match, *this->world_scan_, this->state_.get_transform()); // Not working for PCL 1.12

        if(this->config_.debug){ // save final scan without voxel grid
            raw_world_scan_->points.clear();
            raw_world_scan_->points.resize(deskewed_Xt2_pc_->points.size());
            #pragma omp parallel for num_threads(this->max_num_threads_)
            for(std::size_t i=0; i < deskewed_Xt2_pc_->points.size(); i++){
                LioPointType pt = deskewed_Xt2_pc_->points[i];
                pt.getVector3fMap() = this->state_.get_transform() * pt.getVector3fMap(); 
                raw_world_scan_->points[i] = pt;
            }
            // pcl::transformPointCloud (*deskewed_Xt2_pc_, *this->raw_world_scan_, this->state_.get_transform()); // Not working for PCL 1.12
        }

        // Add scan to map
        {
        GILDA_PROFILE_SCOPE(profiler_, "processScan/Map update");
        auto pose_cov = this->getPoseCovariance();
        map_->update(pc2match_, this->state_, pose_cov);
        }

    }else
        std::cout << "-------------- gilda_lio::NULL ITERATION --------------\n";


    this->prev_scan_stamp_ = this->scan_stamp_;
}

// Compute point-to-plane measurement model
void gilda_lio::OdometryCore::pointToPlaneResidual(const iESEKF::Group& X_now, iESEKF::Measurement& z, iESEKF::HMat& H)
{
    gilda_lio::State st;
    iESEKF::group_to_state(X_now, st);

    auto matches = this->map_->match(st, this->pc2match_);

    using Scalar = iESEKF::Scalar;

    static std::size_t max_matches = static_cast<std::size_t>(config_.max_num_matches);
    
    std::size_t N = (matches.size() > max_matches) ? 
                    max_matches : matches.size();

    H = iESEKF::HMat::Zero(N, iESEKF::Bundle::DoF);
    z.resize(N);

    // For each match, calculate its derivative and distance
    #pragma omp parallel for num_threads(this->max_num_threads_)
    for (std::size_t i = 0; i < N; ++i) {
        Match match = matches[i];

        // Set correct dimension/type
        Eigen::Matrix<Scalar, 3, 1> p_imu, n;
        p_imu = match.local_point.cast<Scalar>();
        n     = match.coeffs.head(3).cast<Scalar>();
        n.normalize();

        // Fill H with state part
        iESEKF::fill_H_point_to_plane(X_now, n, p_imu, i, H);

        // Measurement: point-to-plane distance
        z(i) = -Scalar(match.dist);
    }
}

gilda_lio::SensorType gilda_lio::OdometryCore::get_sensor_type() const
{
    return config_.sensor_type;
}

// Get covariance (6x6 pose + 6x6 twist flattened row-major)
std::vector<double> gilda_lio::OdometryCore::getPoseCovariance() const
{
    return iESEKF::get_pose_covariance(this->filter_->getCovariance());
}

std::vector<double> gilda_lio::OdometryCore::getTwistCovariance() const
{
    std::vector<double> cov = iESEKF::get_velocity_covariance(this->filter_->getCovariance());
    // add gyro covariance
    cov[21] = config_.cov_gyro;
    cov[28] = config_.cov_gyro;
    cov[25] = config_.cov_gyro;
    return cov;
}

const gilda_lio::Map::MapType* gilda_lio::OdometryCore::getMap() const {
    if(map_ == nullptr) return nullptr;
    return map_->getMap();
}

void gilda_lio::OdometryCore::initFilter() {

    iESEKF::Filter::NoiseMatrix Q = iESEKF::Filter::NoiseMatrix::Identity();
    Q.block<3, 3>(0, 0) = static_cast<iESEKF::Scalar>(config_.cov_gyro) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
    Q.block<3, 3>(3, 3) = static_cast<iESEKF::Scalar>(config_.cov_acc) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
    Q.block<3, 3>(6, 6) = static_cast<iESEKF::Scalar>(config_.cov_bias_gyro) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
    Q.block<3, 3>(9, 9) = static_cast<iESEKF::Scalar>(config_.cov_bias_acc) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();

    iESEKF::Filter::MatDoF P = 1.0e-3f * iESEKF::Filter::MatDoF::Identity();

    // Initialize iESEKF
    this->filter_ = std::make_unique<iESEKF::Filter>(
        P,
        Q,
        iESEKF::f,
        iESEKF::df_dx,
        iESEKF::df_dw,
        iESEKF::degeneracy_callback
    );
    this->filter_->setMaxIters(config_.max_ekf_iters);

    this->filter_->setTolerance(config_.ekf_tolerance);
}

void gilda_lio::OdometryCore::initState() {

    this->state_.q.normalize();

    iESEKF::Group group;
    iESEKF::state_to_group(this->state_, group);

    this->filter_->setState(group); // set initial state
}

void gilda_lio::OdometryCore::imuToBody(lie_odyssey::IMUmeas& imu)
{
    double dt = imu.stamp - this->last_imu_stamp_;
    
    if ( (dt <= 0.) || (dt > 0.1) ) { dt = 1.0/200.0; }

    const Eigen::Matrix3f R = config_.imu_extr.rotation();
    const Eigen::Vector3f t = config_.imu_extr.translation();

    // Transform angular velocity (will be the same on a rigid body, so just rotate to baselink frame)
    Eigen::Vector3f ang_vel_cg = R * imu.gyro.cast<float>();

    // Transform linear acceleration (need to account for component due to translational difference)
    Eigen::Vector3f lin_accel_cg = R * imu.accel.cast<float>();

    static Eigen::Vector3f ang_vel_cg_prev = ang_vel_cg;

    lin_accel_cg += ((ang_vel_cg - ang_vel_cg_prev) / dt).cross(-t)
                + ang_vel_cg.cross(ang_vel_cg.cross(-t));

    ang_vel_cg_prev = ang_vel_cg;

    imu.gyro   = ang_vel_cg.cast<double>();
    imu.accel  = lin_accel_cg.cast<double>();
    imu.dt     = dt;

    this->last_imu_stamp_ = imu.stamp;
}

void gilda_lio::OdometryCore::propagateIMU(const lie_odyssey::IMUmeas& imu)
{
    // Propagate IMU measurement
    this->mtx_filter.lock();
    this->filter_->predict(imu);
    this->mtx_filter.unlock();

    // Save propagated state for motion compensation
    gilda_lio::State st;
    iESEKF::group_to_state(this->filter_->getState(), st);
    st.a = imu.accel.cast<float>();
    st.w = imu.gyro.cast<float>();
    st.time = imu.stamp;
    st.bias.a = this->state_.bias.a;
    st.bias.w = this->state_.bias.w;
    this->mtx_state_buf.lock();
    this->state_buffer_.push_front(st);
    this->mtx_state_buf.unlock();

    this->last_propagate_stamp_ = imu.stamp;
}

pcl::PointCloud<LioPointType>::Ptr gilda_lio::OdometryCore::deskewScan(pcl::PointCloud<LioPointType>::Ptr& pc, double& start_time)
{
    GILDA_PROFILE_SCOPE(profiler_, "processScan/deskewScan");

    if(pc->points.size() < 1) 
        return gilda_lio::make_shared<pcl::PointCloud<LioPointType>>();

    // individual point timestamps should be relative to this time
    double sweep_ref_time = start_time;
    bool end_of_sweep = this->config_.end_of_sweep;

    // sort points by timestamp
    std::function<bool(const LioPointType&, const LioPointType&)> point_time_cmp;
    std::function<double(LioPointType&)> extract_point_time;

    if (this->config_.sensor_type == gilda_lio::SensorType::OUSTER) {

        point_time_cmp = [&end_of_sweep](const LioPointType& p1, const LioPointType& p2)
        {   if (end_of_sweep) return p1.t > p2.t; 
            else return p1.t < p2.t; };
        extract_point_time = [&sweep_ref_time, &end_of_sweep](LioPointType& pt)
        {   if (end_of_sweep) return sweep_ref_time - pt.t * 1e-9f; 
            else return sweep_ref_time + pt.t * 1e-9f; };

    } else if (this->config_.sensor_type == gilda_lio::SensorType::VELODYNE) {
        
        point_time_cmp = [&end_of_sweep](const LioPointType& p1, const LioPointType& p2)
        {   if (end_of_sweep) return p1.time > p2.time; 
            else return p1.time < p2.time; };
        extract_point_time = [&sweep_ref_time, &end_of_sweep](LioPointType& pt)
        {   if (end_of_sweep) return sweep_ref_time - pt.time; 
            else return sweep_ref_time + pt.time; };

    } else if (this->config_.sensor_type == gilda_lio::SensorType::HESAI) {

        point_time_cmp = [](const LioPointType& p1, const LioPointType& p2)
        { return p1.timestamp < p2.timestamp; };
        extract_point_time = [](LioPointType& pt)
        { return pt.timestamp; };

    } else if (this->config_.sensor_type == gilda_lio::SensorType::LIVOX) {
        
        point_time_cmp = [](const LioPointType& p1, const LioPointType& p2)
        { return p1.timestamp < p2.timestamp; };
        extract_point_time = [&sweep_ref_time, &end_of_sweep](LioPointType& pt)
        {   if (end_of_sweep) return sweep_ref_time - pt.timestamp * 1e-9f; 
            else return sweep_ref_time + pt.timestamp * 1e-9f; };
    } else {
        std::cout << "-------------------------------------------------------------------\n";
        std::cout << "gilda_lio::FATAL ERROR: LiDAR sensor type unknown or not specified!\n";
        std::cout << "-------------------------------------------------------------------\n";
        return gilda_lio::make_shared<pcl::PointCloud<LioPointType>>();
    }

    // copy points into deskewed_scan in order of timestamp
    pcl::PointCloud<LioPointType>::Ptr deskewed_scan (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>());
    deskewed_scan->points.resize(pc->points.size());

    std::partial_sort_copy(pc->points.begin(), pc->points.end(),
                        deskewed_scan->points.begin(), deskewed_scan->points.end(), point_time_cmp);

    if(deskewed_scan->points.size() < 1){
        std::cout << "gilda_lio::ERROR: failed to sort input pointcloud!\n";
        return gilda_lio::make_shared<pcl::PointCloud<LioPointType>>();
    }

    // compute offset between sweep reference time and IMU data
    double offset = 0.0;
    if (config_.time_offset) {
        offset = this->last_imu_stamp_ - extract_point_time(deskewed_scan->points[deskewed_scan->points.size()-1]) - 1.e-4; // automatic sync (not precise!)
        if(offset > 0.0) offset = 0.0; // don't jump into future
    }

    // Set scan_stamp for next iteration
    this->scan_stamp_ = extract_point_time(deskewed_scan->points[deskewed_scan->points.size()-1]) + offset;

    // If motion compensation is disabled, return sorted pointcloud without deskewing
    if(not this->config_.motion_compensation){

        #pragma omp parallel for num_threads(this->max_num_threads_)
        for (std::size_t k = 0; k < deskewed_scan->points.size(); k++) {
            auto &pt = deskewed_scan->points[k];
            pt.getVector3fMap() = this->config_.lidar_extr * pt.getVector3fMap(); // baselink/body frame
        }

        return deskewed_scan; 
    }

    // IMU prior & deskewing 
    std::vector<gilda_lio::State> frames = this->integrateImu(this->prev_scan_stamp_, this->scan_stamp_); // baselink/body frames

    if(frames.size() < 1){
        std::cout << "gilda_lio::WARNING: No frames obtained from IMU propagation!\n";
        std::cout << "           Scan not deskewed!\n";

        #pragma omp parallel for num_threads(this->max_num_threads_)
        for (std::size_t k = 0; k < deskewed_scan->points.size(); k++) {
            auto &pt = deskewed_scan->points[k];
            pt.getVector3fMap() = this->config_.lidar_extr * pt.getVector3fMap(); // baselink/body frame
        }

        return deskewed_scan; 
    }

    // deskewed pointcloud w.r.t last known state prediction
    pcl::PointCloud<LioPointType>::Ptr deskewed_Xt2_scan_ (gilda_lio::make_shared<pcl::PointCloud<LioPointType>>());
    deskewed_Xt2_scan_->points.resize(deskewed_scan->points.size());

    GILDA_PROFILE_SCOPE(profiler_, "processScan/deskewScan/deskew_points");
    #pragma omp parallel for num_threads(this->max_num_threads_)
    for (std::size_t k = 0; k < deskewed_scan->points.size(); k++) {

        int i_f = gilda_lio::binary_search_tailored(frames, extract_point_time(deskewed_scan->points[k])+offset);

        gilda_lio::State X0 = frames[i_f];
        X0.update(extract_point_time(deskewed_scan->points[k]) + offset);

        Eigen::Isometry3f T = X0.get_transform() * this->config_.lidar_extr;

        // world frame deskewed pc
        auto &pt = deskewed_scan->points[k]; // lidar frame
        pt.getVector3fMap() = T * pt.getVector3fMap(); // world/global frame

        // Xt2 frame deskewed pc
        auto pt2 = deskewed_scan->points[k];
        pt2.getVector3fMap() = this->state_.get_inv_transform() * pt.getVector3fMap(); // Xt2 frame
        pt2.intensity = pt.intensity;

        deskewed_Xt2_scan_->points[k] = pt2;
    }

    if(this->config_.debug) // debug only
        this->deskewed_scan_ = deskewed_scan;

    return deskewed_Xt2_scan_; 
}

std::vector<gilda_lio::State> gilda_lio::OdometryCore::integrateImu(double start_time, double end_time)
{
    GILDA_PROFILE_SCOPE(profiler_, "processScan/deskewScan/integrateImu");

    std::vector<gilda_lio::State> states;

    boost::circular_buffer<gilda_lio::State>::reverse_iterator begin_prop_it;
    boost::circular_buffer<gilda_lio::State>::reverse_iterator end_prop_it;
    if (not this->propagatedFromTimeRange(start_time, end_time, begin_prop_it, end_prop_it)) {
        // not enough IMU measurements, return empty vector
        std::cout << "gilda_lio::propagatedFromTimeRange(): not enough propagated states!\n";
        return states;
    }

    for(auto it = begin_prop_it; it != end_prop_it; it++)
        states.push_back(*it);

    return states;
}

bool gilda_lio::OdometryCore::propagatedFromTimeRange(double start_time, double end_time,
                                        boost::circular_buffer<gilda_lio::State>::reverse_iterator& begin_prop_it,
                                        boost::circular_buffer<gilda_lio::State>::reverse_iterator& end_prop_it) {
    if(this->state_buffer_.empty())
        return false;

    if (this->state_buffer_.front().time < end_time) {
        // Wait for the latest IMU data
        std::cout << "PROPAGATE WAITING...\n";
        std::cout << "     - buffer time: " << std::setprecision(15) << state_buffer_.front().time << std::endl;
        std::cout << "     - end scan time: " << std::setprecision(15) << end_time << std::endl;
        std::unique_lock<decltype(this->mtx_state_buf)> lock(this->mtx_state_buf);
        this->cv_prop_stamp.wait(lock, [this, &end_time]{ return this->state_buffer_.front().time >= end_time; });
    }

    auto prop_it = this->state_buffer_.begin();

    auto last_prop_it = prop_it;
    prop_it++;
    while (prop_it != this->state_buffer_.end() && prop_it->time >= end_time) {
        last_prop_it = prop_it;
        prop_it++;
    }

    while (prop_it != this->state_buffer_.end() && prop_it->time >= start_time) {
        prop_it++;
    }

    if (prop_it == this->state_buffer_.end()) {
        // not enough IMU measurements, return false
        return false;
    }
    prop_it++;

    // Set reverse iterators (to iterate forward in time)
    end_prop_it = boost::circular_buffer<gilda_lio::State>::reverse_iterator(last_prop_it);
    begin_prop_it = boost::circular_buffer<gilda_lio::State>::reverse_iterator(prop_it);

    return true;
}
