#include "lidar_odometry_ros/modules/localizer.hpp"

// class lidar_odometry_ros::Localizer
    // public

        Localizer::Localizer() : scan_stamp(0.0), prev_scan_stamp(0.0), scan_dt(0.1), imu_stamp(0.0), 
                                prev_imu_stamp(0.0), imu_dt(0.005), first_imu_stamp(0.0), last_propagate_time_(-1.0), 
                                imu_calib_time_(3.0), gravity_(9.81), imu_calibrated_(false), numProcessors(0), 
                                deskew_size(0), propagated_size(0)
                            { 

            this->original_scan  = pcl::PointCloud<PointType>::ConstPtr (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
            this->deskewed_scan  = pcl::PointCloud<PointType>::ConstPtr (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
            this->pc2match       = pcl::PointCloud<PointType>::Ptr (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
            this->final_raw_scan = pcl::PointCloud<PointType>::Ptr (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
            this->final_scan     = pcl::PointCloud<PointType>::Ptr (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
        }

        void Localizer::init(Config& cfg){

            // Save config
            this->config = cfg;

            // Set num of threads
            this->num_threads_ = omp_get_max_threads();
            if(num_threads_ > config.num_threads) this->num_threads_ = config.num_threads;

            // Update Mapper config
            lidar_odometry_ros::Mapper& map = lidar_odometry_ros::Mapper::getInstance();
            map.set_num_threads(this->num_threads_);
            map.set_config(this->config.mapping);

            // Initialize Iterated Kalman Filter on Manifolds
            this->init_iESEKF();

            // Set buffer capacity
            this->imu_buffer.set_capacity(2000);
            this->propagated_buffer.set_capacity(2000);

            // PCL filters setup
            this->crop_filter.setNegative(true);
            this->crop_filter.setMin(Eigen::Vector4f(config.filters.cropBoxMin[0], config.filters.cropBoxMin[1], config.filters.cropBoxMin[2], 1.0));
            this->crop_filter.setMax(Eigen::Vector4f(config.filters.cropBoxMax[0], config.filters.cropBoxMax[1], config.filters.cropBoxMax[2], 1.0));

            this->voxel_filter.setLeafSize(config.filters.leafSize[0], config.filters.leafSize[0], config.filters.leafSize[0]);

            // LiDAR sensor type
            this->set_sensor_type(config.sensor_type); 

            // IMU intrinsics
            this->imu_accel_sm_ = Eigen::Map<Eigen::Matrix3f>(config.intrinsics.imu_sm.data(), 3, 3);
            this->state.b.accel = Eigen::Map<Eigen::Vector3f>(config.intrinsics.accel_bias.data(), 3);
            this->state.b.gyro  = Eigen::Map<Eigen::Vector3f>(config.intrinsics.gyro_bias.data(), 3);

            // Extrinsics
            this->extr.imu2baselink.t = Eigen::Map<Eigen::Vector3f>(config.extrinsics.imu2baselink_t.data(), 3);
            Eigen::Matrix3f baselink2imu_R = Eigen::Map<Eigen::Matrix3f>(config.extrinsics.imu2baselink_R.data(), 3, 3);
            this->extr.imu2baselink.R = baselink2imu_R.transpose();

            this->extr.imu2baselink_T = Eigen::Matrix4f::Identity();
            this->extr.imu2baselink_T.block(0, 3, 3, 1) = this->extr.imu2baselink.t;
            this->extr.imu2baselink_T.block(0, 0, 3, 3) = this->extr.imu2baselink.R;

            this->extr.lidar2baselink.t = Eigen::Map<Eigen::Vector3f>(config.extrinsics.lidar2baselink_t.data(), 3);
            Eigen::Matrix3f baselink2lidar_R = Eigen::Map<Eigen::Matrix3f>(config.extrinsics.lidar2baselink_R.data(), 3, 3);
            this->extr.lidar2baselink.R = baselink2lidar_R.transpose();

            this->extr.lidar2baselink_T = Eigen::Matrix4f::Identity();
            this->extr.lidar2baselink_T.block(0, 3, 3, 1) = this->extr.lidar2baselink.t;
            this->extr.lidar2baselink_T.block(0, 0, 3, 3) = this->extr.lidar2baselink.R;

            // Avoid unnecessary warnings from PCL
            pcl::console::setVerbosityLevel(pcl::console::L_ERROR);

            // Initial calibration
            if( not (config.gravity_align || config.calibrate_accel || config.calibrate_gyro) ){ // no need for automatic calibration
                this->init_iESEKF_state();
                this->imu_calibrated_ = true;
            }

            // Calibration time
            this->imu_calib_time_ = config.imu_calib_time;

            // CPU info
            this->getCPUinfo();

            // Set up buffer capacities
            this->imu_rates.set_capacity(1000);
            this->lidar_rates.set_capacity(1000);
            this->cpu_times.set_capacity(1000);
            this->cpu_percents.set_capacity(1000);

            // Fill CPU stats
            this->cpu_time = 0.0f;
            this->cpu_max_time = 0.0f;
            this->cpu_mean_time = 0.0f;
            this->cpu_cores = 0.0f;
            this->cpu_load = 0.0f;
            this->cpu_max_load = 0.0f;
            this->ram_usage = 0.0f;
        }

        pcl::PointCloud<PointType>::Ptr Localizer::get_pointcloud(){
            return this->final_scan;
        }

        pcl::PointCloud<PointType>::Ptr Localizer::get_finalraw_pointcloud(){
            return this->final_raw_scan;
        }

        pcl::PointCloud<PointType>::ConstPtr Localizer::get_orig_pointcloud(){
            return this->original_scan;
        }

        pcl::PointCloud<PointType>::ConstPtr Localizer::get_deskewed_pointcloud(){
            return this->deskewed_scan;
        }

        pcl::PointCloud<PointType>::Ptr Localizer::get_pc2match_pointcloud(){
            return this->pc2match;
        }

        Matches& Localizer::get_matches(){
            return this->matches;
        }

        bool Localizer::is_calibrated(){
            return this->imu_calibrated_;
        }

        void Localizer::set_sensor_type(uint8_t type){
            if(type < 5)
                this->sensor = static_cast<lidar_odometry_ros::SensorType>(type);
            else 
                this->sensor = lidar_odometry_ros::SensorType::UNKNOWN;
        }

        lidar_odometry_ros::SensorType Localizer::get_sensor_type(){
            return this->sensor;
        }

        State Localizer::getBodyState(){

            if(not this->is_calibrated())
                return State();

            State out = lidar_odometry_ros::State(this->_iESEKF->getState());

            out.w    = this->last_imu.ang_vel;                      // set last IMU meas
            out.a    = this->last_imu.lin_accel;                    // set last IMU meas
            out.time = this->imu_stamp;                             // set current time stamp 

            Eigen::Quaternionf qLI(this->extr.lidar2baselink.R.transpose());    // current orientation in LiDAR frame
            out.p -= this->extr.lidar2baselink.t;                   // position in LiDAR frame
            out.q *= qLI;                                           // attitude in LiDAR frame
            out.v = out.q.toRotationMatrix().transpose() * out.v;   // local velocity vector
            return out;
        }

        State Localizer::getWorldState(){

            if(not this->is_calibrated())
                return State();

            State out = lidar_odometry_ros::State(this->_iESEKF->getState());

            out.w    = this->last_imu.ang_vel;                      // set last IMU meas
            out.a    = this->last_imu.lin_accel;                    // set last IMU meas
            out.time = this->imu_stamp;                             // set current time stamp 

            out.v = out.q.toRotationMatrix().transpose() * out.v;   // local velocity vector

            return out;
        }

        double Localizer::get_propagate_time(){
            return this->last_propagate_time_;
        }

        void Localizer::get_cpu_stats(float &comput_time, float &max_comput_time, float &mean_comput_time,
                            float &cpu_cores, float &cpu_load, float &cpu_max_load, float &ram_usage){
            
            this->mtx_cpu_stats.lock();
            comput_time = this->cpu_time;
            max_comput_time = this->cpu_max_time;
            mean_comput_time = this->cpu_mean_time;
            cpu_cores = this->cpu_cores;
            cpu_load = this->cpu_load;
            cpu_max_load = this->cpu_max_load;
            ram_usage = this->ram_usage;
            this->mtx_cpu_stats.unlock();
        }

        std::vector<double> Localizer::getPoseCovariance(){
            if(not this->is_calibrated())
                return std::vector<double>(36, 0);

            auto P = this->_iESEKF->getCovariance();
            Eigen::Matrix<double, 6, 6> P_pose;
            P_pose.block<3, 3>(0, 0) = P.block<3, 3>(3, 3).cast<double>();
            P_pose.block<3, 3>(0, 3) = P.block<3, 3>(3, 0).cast<double>();
            P_pose.block<3, 3>(3, 0) = P.block<3, 3>(0, 3).cast<double>();
            P_pose.block<3, 3>(3, 3) = P.block<3, 3>(0, 0).cast<double>();

            std::vector<double> cov(P_pose.size());
            Eigen::Map<Eigen::MatrixXd>(cov.data(), P_pose.rows(), P_pose.cols()) = P_pose;

            return cov;
        }

        std::vector<double> Localizer::getTwistCovariance(){
            if(not this->is_calibrated())
                return std::vector<double>(36, 0);

            auto P = this->_iESEKF->getCovariance();
            Eigen::Matrix<double, 6, 6> P_odom = Eigen::Matrix<double, 6, 6>::Zero();
            P_odom.block<3, 3>(0, 0) = P.block<3, 3>(6, 6).cast<double>();
            P_odom.block<3, 3>(3, 3) = config.ekf.cov_gyro * Eigen::Matrix<double, 3, 3>::Identity();

            std::vector<double> cov(P_odom.size());
            Eigen::Map<Eigen::MatrixXd>(cov.data(), P_odom.rows(), P_odom.cols()) = P_odom;

            return cov;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////           Principal callbacks/threads        /////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::updatePointCloud(pcl::PointCloud<PointType>::Ptr& raw_pc, double time_stamp){

            auto start_time = std::chrono::system_clock::now();

            if(raw_pc->points.size() < 1){
                std::cout << "lidar_odometry_ros::Raw PointCloud is empty!\n";
                return;
            }

            if(!this->imu_calibrated_)
                return;

            if(this->imu_buffer.empty()){
                std::cout << "lidar_odometry_ros::IMU buffer is empty!\n";
                return;
            }

            // Remove NaNs
            std::vector<int> idx;
            raw_pc->is_dense = false;
            pcl::removeNaNFromPointCloud(*raw_pc, *raw_pc, idx);

            // Crop Box Filter (1 m^2)
            if(this->config.filters.crop_active){
                this->crop_filter.setInputCloud(raw_pc);
                this->crop_filter.filter(*raw_pc);
            }

            // Distance & Time Rate filters
            static float min_dist = static_cast<float>(this->config.filters.min_dist);
            static int rate_value = this->config.filters.rate_value;
            std::function<bool(boost::range::index_value<PointType&, long>)> filter_f;
            
            if(this->config.filters.dist_active && this->config.filters.rate_active){
                filter_f = [this](boost::range::index_value<PointType&, long> p)
                    { return (Eigen::Vector3f(p.value().x, p.value().y, p.value().z).norm() > min_dist)
                                && (p.index()%rate_value == 0) && this->isInRange(p.value()); };
            }
            else if(this->config.filters.dist_active){
                filter_f = [this](boost::range::index_value<PointType&, long> p)
                    { return (Eigen::Vector3f(p.value().x, p.value().y, p.value().z).norm() > min_dist) &&
                                this->isInRange(p.value()); };
            }
            else if(this->config.filters.rate_active){
                filter_f = [this](boost::range::index_value<PointType&, long> p)
                    { return (p.index()%rate_value == 0) && this->isInRange(p.value()); };
            }else{
                filter_f = [this](boost::range::index_value<PointType&, long> p)
                    { return this->isInRange(p.value()); };
            }
            auto filtered_pc = raw_pc->points 
                        | boost::adaptors::indexed()
                        | boost::adaptors::filtered(filter_f);

            pcl::PointCloud<PointType>::Ptr input_pc (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
            for (auto it = filtered_pc.begin(); it != filtered_pc.end(); it++) {
                input_pc->points.push_back(it->value());
            }

            if(this->config.debug) // debug only
                this->original_scan = lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>(*input_pc); // LiDAR frame

            // Motion compensation
            pcl::PointCloud<PointType>::Ptr deskewed_Xt2_pc_ (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
            deskewed_Xt2_pc_ = this->deskewPointCloud(input_pc, time_stamp);
            /*NOTE: deskewed_Xt2_pc_ should be in base_link/body frame w.r.t last propagated state (Xt2) */

            // Voxel Grid Filter
            if (this->config.filters.voxel_active) { 
                pcl::PointCloud<PointType>::Ptr current_scan_
                    (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>(*deskewed_Xt2_pc_));
                this->voxel_filter.setInputCloud(current_scan_);
                this->voxel_filter.filter(*current_scan_);
                this->pc2match = current_scan_;
            } else {
                this->pc2match = deskewed_Xt2_pc_;
            }

            if(this->pc2match->points.size() > 1){

                // iESEKF observation stage
                this->mtx_iesekf.lock();

                // Call Mapper obj
                lidar_odometry_ros::Mapper& map = lidar_odometry_ros::Mapper::getInstance();

                // Update iESEKF measurements 
                this->_iESEKF->update
                        <iESEKF::Measurement, 
                        iESEKF::HMat> (static_cast<iESEKF::Scalar>(config.mapping.LIDAR_NOISE) /*LiDAR noise*/,
                                        iESEKF::H_fun /*Measurement function*/);
                /*NOTE: update() will trigger the matching procedure ( see "iESEKF.cpp" )
                in order to update the measurement stage of the KF with the computed point-to-plane distances*/

                map.clear_matches(); // clear matches vector for next iteration

                    // Get output state from iESEKF
                lidar_odometry_ros::State corrected_state = lidar_odometry_ros::State(this->_iESEKF->getState(), 
                                                                                      this->scan_stamp);

                // Set estimated biases & gravity to constant
                if(this->config.calibrate_gyro)  corrected_state.b.gyro  = this->state.b.gyro;
                if(this->config.calibrate_accel) corrected_state.b.accel = this->state.b.accel;
                if(this->config.gravity_align)   corrected_state.g       = this->state.g;

                // Update current state estimate
                this->state      = corrected_state;
                this->state.w    = this->last_imu.ang_vel;
                this->state.a    = this->last_imu.lin_accel;

                this->mtx_iesekf.unlock();

                // Transform deskewed pc 
                    // Get deskewed scan to add to map
                pcl::PointCloud<PointType>::Ptr mapped_scan (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
                mapped_scan->points.resize(pc2match->points.size());

                // pcl::transformPointCloud (*this->pc2match, *mapped_scan, this->state.get_RT()); // Not working for PCL 1.12
                #pragma omp parallel for num_threads(this->num_threads_)
                for(std::size_t i=0; i < pc2match->points.size(); i++){
                    PointType pt = pc2match->points[i];
                    pt.getVector4fMap()[3] = 1.;
                    pt.getVector4fMap() = this->state.get_RT() * pt.getVector4fMap(); 
                    mapped_scan->points[i] = pt;
                    /*NOTE: pc2match must be in base_link frame w.r.t Xt2 frame for this transform to work.
                        mapped_scan is in world/global frame.
                    */
                }

                    // Get final scan to output (in world/global frame)
                this->final_scan = mapped_scan; // mapped_scan = final_scan (for now)
                // pcl::transformPointCloud (*this->pc2match, *this->final_scan, this->state.get_RT()); // Not working for PCL 1.12

                if(this->config.debug){ // save final scan without voxel grid
                    final_raw_scan->points.clear();
                    final_raw_scan->points.resize(deskewed_Xt2_pc_->points.size());
                    #pragma omp parallel for num_threads(this->num_threads_)
                    for(std::size_t i=0; i < deskewed_Xt2_pc_->points.size(); i++){
                        PointType pt = deskewed_Xt2_pc_->points[i];
                        pt.getVector4fMap()[3] = 1.;
                        pt.getVector4fMap() = this->state.get_RT() * pt.getVector4fMap(); 
                        final_raw_scan->points[i] = pt;
                    }
                    // pcl::transformPointCloud (*deskewed_Xt2_pc_, *this->final_raw_scan, this->state.get_RT()); // Not working for PCL 1.12
                }

                // Add scan to map
                map.add(mapped_scan, this->scan_stamp);

            }else
                std::cout << "-------------- lidar_odometry_ros::NULL ITERATION --------------\n";

            auto end_time = std::chrono::system_clock::now();
            elapsed_time = end_time - start_time;

            // fill stats
            if(this->prev_scan_stamp > 0.0) this->lidar_rates.push_front( 1. / (this->scan_stamp - this->prev_scan_stamp) );
            if(calibrating > 0) this->cpu_times.push_front(elapsed_time.count());
            else{
                this->cpu_times.push_front(0.0);
                calibrating++;
            }
            // if(calibrating < UCHAR_MAX) calibrating++;

            // debug thread
            this->debug_thread = std::thread( &Localizer::debugVerbose, this );
            this->debug_thread.detach();

            this->prev_scan_stamp = this->scan_stamp;
        }

        void Localizer::updateIMU(IMUmeas& raw_imu){

            this->imu_stamp = raw_imu.stamp;
            IMUmeas imu = this->imu2baselink(raw_imu);

            if(this->first_imu_stamp == 0.0)
                this->first_imu_stamp = imu.stamp;

            this->imu_rates.push_front( 1./imu.dt );

            // IMU calibration procedure - do only while the robot is in stand still!
            if (not this->imu_calibrated_) {

                static int num_samples = 0;
                static Eigen::Vector3f gyro_avg (0., 0., 0.);
                static Eigen::Vector3f accel_avg (0., 0., 0.);
                static bool print = true;

                if ((imu.stamp - this->first_imu_stamp) < this->imu_calib_time_) {

                    num_samples++;

                    gyro_avg[0] += imu.ang_vel[0];
                    gyro_avg[1] += imu.ang_vel[1];
                    gyro_avg[2] += imu.ang_vel[2];

                    accel_avg[0] += imu.lin_accel[0];
                    accel_avg[1] += imu.lin_accel[1];
                    accel_avg[2] += imu.lin_accel[2];

                    if(print) {
                        std::cout << std::endl << " Calibrating IMU for " << this->imu_calib_time_ << " seconds... \n";
                        std::cout.flush();
                        print = false;
                    }

                } else {

                    gyro_avg /= num_samples;
                    accel_avg /= num_samples;

                    Eigen::Vector3f grav_vec (0., 0., this->gravity_);

                    this->state.q = imu.q;
                    this->state.g = grav_vec;

                    if (this->config.gravity_align) {

                        std::cout << " Accel mean: " << "[ " << accel_avg[0] << ", " << accel_avg[1] << ", " << accel_avg[2] << " ]\n";

                        // Estimate gravity vector - Only approximate if biases have not been pre-calibrated
                        grav_vec = (accel_avg - this->state.b.accel).normalized() * abs(this->gravity_);
                        Eigen::Quaternionf grav_q = Eigen::Quaternionf::FromTwoVectors(grav_vec, Eigen::Vector3f(0., 0., this->gravity_));
                        
                        std::cout << " Gravity mean: " << "[ " << grav_vec[0] << ", " << grav_vec[1] << ", " << grav_vec[2] << " ]\n";

                        // set gravity aligned orientation
                        this->state.q = grav_q;

                        // set estimated gravity vector
                        this->state.g = grav_vec;

                    }

                    if (this->config.calibrate_accel) {

                        // subtract gravity from avg accel to get bias
                        this->state.b.accel = accel_avg - grav_vec;

                        std::cout << " Accel biases [xyz]: " << to_string_with_precision(this->state.b.accel[0], 8) << ", "
                                                            << to_string_with_precision(this->state.b.accel[1], 8) << ", "
                                                            << to_string_with_precision(this->state.b.accel[2], 8) << std::endl;
                    }

                    if (this->config.calibrate_gyro) {

                        this->state.b.gyro = gyro_avg;

                        std::cout << " Gyro biases  [xyz]: " << to_string_with_precision(this->state.b.gyro[0], 8) << ", "
                                                            << to_string_with_precision(this->state.b.gyro[1], 8) << ", "
                                                            << to_string_with_precision(this->state.b.gyro[2], 8) << std::endl;
                    }

                    this->state.q.normalize();

                    // Set calib flag
                    this->imu_calibrated_ = true;

                    // Set initial KF state
                    this->init_iESEKF_state();

                    // Initial attitude
                    auto euler = this->state.q.toRotationMatrix().eulerAngles(2, 1, 0);
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

                // Apply the calibrated bias to the new IMU measurements
                Eigen::Vector3f lin_accel_corrected = (this->imu_accel_sm_ * imu.lin_accel) - this->state.b.accel;
                Eigen::Vector3f ang_vel_corrected = imu.ang_vel - this->state.b.gyro;

                imu.lin_accel = lin_accel_corrected;
                imu.ang_vel   = ang_vel_corrected;

                this->last_imu = imu;

                // Store calibrated IMU measurements into imu buffer for manual integration later.
                this->imu_buffer.push_front(imu);

                // iESEKF propagate state
                this->propagateImu(imu);
                this->cv_prop_stamp.notify_one(); // Notify PointCloud thread that propagated IMU data exists for this time
            }

        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////          KF measurement model        /////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::calculate_H(const iESEKF::Group& group, const Matches& matches, iESEKF::Measurement& z, iESEKF::HMat& H)
        {
            using Scalar = iESEKF::Scalar;

            static std::size_t max_matches = static_cast<std::size_t>(config.mapping.MAX_NUM_MATCHES);
            
            std::size_t N = (matches.size() > max_matches) ? 
                            max_matches : matches.size();

            this->num_matches = N;

            H = iESEKF::HMat::Zero(N, iESEKF::Bundle::DoF);
            z.resize(N);

            State current_state(group);

            // For each match, calculate its derivative and distance
            #pragma omp parallel for num_threads(this->num_threads_)
            for (std::size_t i = 0; i < N; ++i) {
                Match match = matches[i];
                Eigen::Vector4f p4_imu = match.get_4Dlocal();
                Eigen::Vector4f normal = match.plane.get_normal();

                // match.update_global(current_state); // update match global position using current state estimate

                // Set correct dimension/type
                Eigen::Matrix<Scalar, 3, 1> p_imu, n;
                p_imu = p4_imu.head(3).cast<Scalar>();
                n     = normal.head(3).cast<Scalar>();
                n.normalize();

                // Fill H with state part
                iESEKF::fill_H_point_to_plane(group, n, p_imu, i, H);
     
                // Measurement: distance to the closest plane
                z(i) = -Scalar(match.get_distance());
            }

            if(this->config.debug) 
                this->matches = matches;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////          KF propagation model        /////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::propagateImu(const IMUmeas& imu){
            lie_odyssey::IMUmeas input;
            input.accel = imu.lin_accel.cast<double>();
            input.gyro = imu.ang_vel.cast<double>();
            input.dt = imu.dt;

            // Propagate IMU measurement
            this->mtx_iesekf.lock();
            this->_iESEKF->predict(input);
            this->mtx_iesekf.unlock();

            // Save propagated state for motion compensation
            this->mtx_prop.lock();
            this->propagated_buffer.push_front( lidar_odometry_ros::State(this->_iESEKF->getState(), 
                                                                imu.stamp, imu.lin_accel, imu.ang_vel)
                                                );
            this->mtx_prop.unlock();

            this->last_propagate_time_ = imu.stamp;
        }

        void Localizer::propagateImu(double t1, double t2){

            boost::circular_buffer<IMUmeas>::reverse_iterator begin_imu_it;
            boost::circular_buffer<IMUmeas>::reverse_iterator end_imu_it;
            if (not this->imuMeasFromTimeRange(t1, t2, begin_imu_it, end_imu_it)) {
                // not enough IMU measurements, return empty vector
                std::cout << "lidar_odometry_ros::propagateImu(): not enough IMU measurements\n";
                return;
            }

            // Iterate over IMU measurements
            auto imu_it = begin_imu_it;

            // Propagate IMU meas and save it for motion compensation
            this->mtx_iesekf.lock();
            this->mtx_prop.lock();

            lie_odyssey::IMUmeas input;
            for (; imu_it != end_imu_it; imu_it++) {
                const IMUmeas& imu = *imu_it;

                input.accel = imu.lin_accel.cast<double>();
                input.gyro = imu.ang_vel.cast<double>();
                input.dt = imu.dt;

                this->_iESEKF->predict(input);
                this->propagated_buffer.push_front( lidar_odometry_ros::State(this->_iESEKF->getState(), 
                                                                imu.stamp, imu.lin_accel, imu.ang_vel)
                                                    );
            }

            this->mtx_iesekf.unlock();
            this->mtx_prop.unlock();

            this->last_propagate_time_ = end_imu_it->stamp;
        }

    // private

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        /////////////////////////////////          Aux. functions        ///////////////////////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        void Localizer::init_iESEKF() {

            iESEKF::Filter::NoiseMatrix Q = iESEKF::Filter::NoiseMatrix::Identity();
            Q.block<3, 3>(0, 0) = static_cast<iESEKF::Scalar>(config.ekf.cov_gyro) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
            Q.block<3, 3>(3, 3) = static_cast<iESEKF::Scalar>(config.ekf.cov_acc) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
            Q.block<3, 3>(6, 6) = static_cast<iESEKF::Scalar>(config.ekf.cov_bias_gyro) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
            Q.block<3, 3>(9, 9) = static_cast<iESEKF::Scalar>(config.ekf.cov_bias_acc) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();

            iESEKF::Filter::MatDoF P = 1.0e-3f * iESEKF::Filter::MatDoF::Identity();

            // Initialize iESEKF
            this->_iESEKF = std::make_unique<iESEKF::Filter>(
                P,
                Q,
                iESEKF::f,
                iESEKF::df_dx,
                iESEKF::df_dw,
                iESEKF::degeneracy_callback
            );
            this->_iESEKF->setMaxIters(config.ekf.MAX_NUM_ITERS);

            this->_iESEKF->setTolerance(config.ekf.TOLERANCE);
        }

        void Localizer::init_iESEKF_state() {

            using V3 = iESEKF::V3;
            using Quat = iESEKF::Quat;

            V3 gravity = (this->imu_calibrated_) ? 
                            this->state.g.cast<iESEKF::Scalar>() : 
                            V3(0., 0., static_cast<iESEKF::Scalar>(this->gravity_));

            V3 imu_p = this->extr.imu2baselink.t.cast<iESEKF::Scalar>();              // position of IMU w.r.t baselink
            Quat imu_q(this->extr.imu2baselink.R.transpose().cast<iESEKF::Scalar>()); // orientation of IMU w.r.t baselink
            imu_q.normalize();

            auto X0_group = iESEKF::get_filled_state(imu_p, imu_q, {0., 0., 0.},
                                                    this->state.b.gyro.cast<iESEKF::Scalar>(), 
                                                    this->state.b.accel.cast<iESEKF::Scalar>(), 
                                                    gravity);

            this->_iESEKF->setState(X0_group); // set initial state
        }

        IMUmeas Localizer::imu2baselink(IMUmeas& imu){

            IMUmeas imu_baselink;

            double dt = imu.stamp - this->prev_imu_stamp;
            
            if ( (dt == 0.) || (dt > 0.1) ) { dt = 1.0/200.0; }

            // Transform angular velocity (will be the same on a rigid body, so just rotate to baselink frame)
            Eigen::Vector3f ang_vel_cg = this->extr.imu2baselink.R * imu.ang_vel;

            static Eigen::Vector3f ang_vel_cg_prev = ang_vel_cg;

            // Transform linear acceleration (need to account for component due to translational difference)
            Eigen::Vector3f lin_accel_cg = this->extr.imu2baselink.R * imu.lin_accel;

            lin_accel_cg = lin_accel_cg
                            + ((ang_vel_cg - ang_vel_cg_prev) / dt).cross(-this->extr.imu2baselink.t)
                            + ang_vel_cg.cross(ang_vel_cg.cross(-this->extr.imu2baselink.t));

            ang_vel_cg_prev = ang_vel_cg;

            imu_baselink.ang_vel   = ang_vel_cg;
            imu_baselink.lin_accel = lin_accel_cg;
            imu_baselink.dt        = dt;
            imu_baselink.stamp     = imu.stamp;

            Eigen::Quaternionf q(this->extr.imu2baselink.R);
            q.normalize();
            imu_baselink.q = q * imu.q;

            this->prev_imu_stamp = imu.stamp;

            return imu_baselink;

        }

        pcl::PointCloud<PointType>::Ptr
        Localizer::deskewPointCloud(pcl::PointCloud<PointType>::Ptr& pc, double& start_time){

            if(pc->points.size() < 1) 
                return lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>();

            // individual point timestamps should be relative to this time
            double sweep_ref_time = start_time;
            bool end_of_sweep = this->config.end_of_sweep;

            // sort points by timestamp
            std::function<bool(const PointType&, const PointType&)> point_time_cmp;
            std::function<double(PointType&)> extract_point_time;

            if (this->sensor == lidar_odometry_ros::SensorType::OUSTER) {

                point_time_cmp = [&end_of_sweep](const PointType& p1, const PointType& p2)
                {   if (end_of_sweep) return p1.t > p2.t; 
                    else return p1.t < p2.t; };
                extract_point_time = [&sweep_ref_time, &end_of_sweep](PointType& pt)
                {   if (end_of_sweep) return sweep_ref_time - pt.t * 1e-9f; 
                    else return sweep_ref_time + pt.t * 1e-9f; };

            } else if (this->sensor == lidar_odometry_ros::SensorType::VELODYNE) {
                
                point_time_cmp = [&end_of_sweep](const PointType& p1, const PointType& p2)
                {   if (end_of_sweep) return p1.time > p2.time; 
                    else return p1.time < p2.time; };
                extract_point_time = [&sweep_ref_time, &end_of_sweep](PointType& pt)
                {   if (end_of_sweep) return sweep_ref_time - pt.time; 
                    else return sweep_ref_time + pt.time; };

            } else if (this->sensor == lidar_odometry_ros::SensorType::HESAI) {

                point_time_cmp = [](const PointType& p1, const PointType& p2)
                { return p1.timestamp < p2.timestamp; };
                extract_point_time = [](PointType& pt)
                { return pt.timestamp; };

            } else if (this->sensor == lidar_odometry_ros::SensorType::LIVOX) {
                
                point_time_cmp = [](const PointType& p1, const PointType& p2)
                { return p1.timestamp < p2.timestamp; };
                extract_point_time = [&sweep_ref_time, &end_of_sweep](PointType& pt)
                {   if (end_of_sweep) return sweep_ref_time - pt.timestamp * 1e-9f; 
                    else return sweep_ref_time + pt.timestamp * 1e-9f; };
            } else {
                std::cout << "-------------------------------------------------------------------\n";
                std::cout << "lidar_odometry_ros::FATAL ERROR: LiDAR sensor type unknown or not specified!\n";
                std::cout << "-------------------------------------------------------------------\n";
                return lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>();
            }

            // copy points into deskewed_scan_ in order of timestamp
            pcl::PointCloud<PointType>::Ptr deskewed_scan_ (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
            deskewed_scan_->points.resize(pc->points.size());
            
            std::partial_sort_copy(pc->points.begin(), pc->points.end(),
                                    deskewed_scan_->points.begin(), deskewed_scan_->points.end(), point_time_cmp);

            if(deskewed_scan_->points.size() < 1){
                std::cout << "lidar_odometry_ros::ERROR: failed to sort input pointcloud!\n";
                return lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>();
            }

            // compute offset between sweep reference time and IMU data
            double offset = 0.0;
            if (config.time_offset) {
                offset = this->imu_stamp - extract_point_time(deskewed_scan_->points[deskewed_scan_->points.size()-1]) - 1.e-4; // automatic sync (not precise!)
                if(offset > 0.0) offset = 0.0; // don't jump into future
            }

            // Set scan_stamp for next iteration
            this->scan_stamp = extract_point_time(deskewed_scan_->points[deskewed_scan_->points.size()-1]) + offset;

            // If motion compensation is disabled, return sorted pointcloud without deskewing
            if(not this->config.motion_compensation){
                this->deskew_size = deskewed_scan_->points.size(); // debug info

                #pragma omp parallel for num_threads(this->num_threads_)
                for (std::size_t k = 0; k < deskewed_scan_->points.size(); k++) {
                    auto &pt = deskewed_scan_->points[k];
                    pt.getVector4fMap()[3] = 1.;
                    pt.getVector4fMap() = this->extr.lidar2baselink_T * pt.getVector4fMap(); // baselink/body frame
                }

                return deskewed_scan_; 
            }

            // IMU prior & deskewing 
            States frames = this->integrateImu(this->prev_scan_stamp, this->scan_stamp); // baselink/body frames

            if(frames.size() < 1){
                std::cout << "lidar_odometry_ros::ERROR: No frames obtained from IMU propagation!\n";
                std::cout << "           Returning null deskewed pointcloud!\n";
                return lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>();
            }

            // deskewed pointcloud w.r.t last known state prediction
            pcl::PointCloud<PointType>::Ptr deskewed_Xt2_scan_ (lidar_odometry_ros::make_shared<pcl::PointCloud<PointType>>());
            deskewed_Xt2_scan_->points.resize(deskewed_scan_->points.size());

            this->last_state = lidar_odometry_ros::State(this->_iESEKF->getState()); // baselink/body frame

            #pragma omp parallel for num_threads(this->num_threads_)
            for (std::size_t k = 0; k < deskewed_scan_->points.size(); k++) {

                int i_f = algorithms::binary_search_tailored(frames, extract_point_time(deskewed_scan_->points[k])+offset);

                State X0 = frames[i_f];
                X0.update(extract_point_time(deskewed_scan_->points[k]) + offset);

                Eigen::Matrix4f T = X0.get_RT() * this->extr.lidar2baselink_T;

                // world frame deskewed pc
                auto &pt = deskewed_scan_->points[k]; // lidar frame
                pt.getVector4fMap()[3] = 1.;
                pt.getVector4fMap() = T * pt.getVector4fMap(); // world/global frame

                // Xt2 frame deskewed pc
                auto pt2 = deskewed_scan_->points[k];
                pt2.getVector4fMap() = this->last_state.get_RT_inv() * pt.getVector4fMap(); // Xt2 frame
                pt2.intensity = pt.intensity;

                deskewed_Xt2_scan_->points[k] = pt2;
            }

            // debug info
            this->deskew_size = deskewed_Xt2_scan_->points.size(); 
            this->propagated_size = frames.size();

            if(this->config.debug && this->deskew_size > 0) // debug only
                this->deskewed_scan = deskewed_scan_;

            return deskewed_Xt2_scan_; 
        }

        States Localizer::integrateImu(double start_time, double end_time){

            States imu_se3;

            boost::circular_buffer<State>::reverse_iterator begin_prop_it;
            boost::circular_buffer<State>::reverse_iterator end_prop_it;
            if (not this->propagatedFromTimeRange(start_time, end_time, begin_prop_it, end_prop_it)) {
                // not enough IMU measurements, return empty vector
                std::cout << "lidar_odometry_ros::propagatedFromTimeRange(): not enough propagated states!\n";
                return imu_se3;
            }

            for(auto it = begin_prop_it; it != end_prop_it; it++)
                imu_se3.push_back(*it);

            return imu_se3;
        }

        bool Localizer::isInRange(PointType& p){
            if(not this->config.filters.fov_active) return true;
            return fabs(atan2(p.y, p.x)) < this->config.filters.fov_angle;
        }

        bool Localizer::propagatedFromTimeRange(double start_time, double end_time,
                                                boost::circular_buffer<State>::reverse_iterator& begin_prop_it,
                                                boost::circular_buffer<State>::reverse_iterator& end_prop_it) {

            if (this->propagated_buffer.empty() || this->propagated_buffer.front().time < end_time) {
                // Wait for the latest IMU data
                std::cout << "PROPAGATE WAITING...\n";
                std::cout << "     - buffer time: " << std::setprecision(15) << propagated_buffer.front().time << std::endl;
                std::cout << "     - end scan time: " << std::setprecision(15) << end_time << std::endl;
                std::unique_lock<decltype(this->mtx_prop)> lock(this->mtx_prop);
                this->cv_prop_stamp.wait(lock, [this, &end_time]{ return this->propagated_buffer.front().time >= end_time; });
            }

            auto prop_it = this->propagated_buffer.begin();

            auto last_prop_it = prop_it;
            prop_it++;
            while (prop_it != this->propagated_buffer.end() && prop_it->time >= end_time) {
                last_prop_it = prop_it;
                prop_it++;
            }

            while (prop_it != this->propagated_buffer.end() && prop_it->time >= start_time) {
                prop_it++;
            }

            if (prop_it == this->propagated_buffer.end()) {
                // not enough IMU measurements, return false
                return false;
            }
            prop_it++;

            // Set reverse iterators (to iterate forward in time)
            end_prop_it = boost::circular_buffer<State>::reverse_iterator(last_prop_it);
            begin_prop_it = boost::circular_buffer<State>::reverse_iterator(prop_it);

            return true;
        }

        bool Localizer::imuMeasFromTimeRange(double start_time, double end_time,
                                                boost::circular_buffer<IMUmeas>::reverse_iterator& begin_imu_it,
                                                boost::circular_buffer<IMUmeas>::reverse_iterator& end_imu_it) {

            if (this->imu_buffer.empty() || this->imu_buffer.front().stamp < end_time) {
                return false;
            }

            auto imu_it = this->imu_buffer.begin();

            auto last_imu_it = imu_it;
            imu_it++;
            while (imu_it != this->imu_buffer.end() && imu_it->stamp >= end_time) {
                last_imu_it = imu_it;
                imu_it++;
            }

            while (imu_it != this->imu_buffer.end() && imu_it->stamp >= start_time) {
                imu_it++;
            }

            if (imu_it == this->imu_buffer.end()) {
                // not enough IMU measurements, return false
                return false;
            }
            imu_it++;

            // Set reverse iterators (to iterate forward in time)
            end_imu_it = boost::circular_buffer<IMUmeas>::reverse_iterator(last_imu_it);
            begin_imu_it = boost::circular_buffer<IMUmeas>::reverse_iterator(imu_it);

            return true;
        }

        void Localizer::getCPUinfo(){ // CPU Specs
            char CPUBrandString[0x40];
            memset(CPUBrandString, 0, sizeof(CPUBrandString));

            this->cpu_type = "";

            #ifdef HAS_CPUID
            unsigned int CPUInfo[4] = {0,0,0,0};
            __cpuid(0x80000000, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
            unsigned int nExIds = CPUInfo[0];
            for (unsigned int i = 0x80000000; i <= nExIds; ++i) {
                __cpuid(i, CPUInfo[0], CPUInfo[1], CPUInfo[2], CPUInfo[3]);
                if (i == 0x80000002)
                memcpy(CPUBrandString, CPUInfo, sizeof(CPUInfo));
                else if (i == 0x80000003)
                memcpy(CPUBrandString + 16, CPUInfo, sizeof(CPUInfo));
                else if (i == 0x80000004)
                memcpy(CPUBrandString + 32, CPUInfo, sizeof(CPUInfo));
            }
            this->cpu_type = CPUBrandString;
            boost::trim(this->cpu_type);
            #endif

            FILE* file;
            struct tms timeSample;
            char line[128];

            this->lastCPU = times(&timeSample);
            this->lastSysCPU = timeSample.tms_stime;
            this->lastUserCPU = timeSample.tms_utime;

            file = fopen("/proc/cpuinfo", "r");
            this->numProcessors = 0;
            while(fgets(line, 128, file) != nullptr) {
                if (strncmp(line, "processor", 9) == 0) this->numProcessors++;
            }
            fclose(file);
        }

        void Localizer::debugVerbose(){

            // Average computation time
            double avg_comp_time =
                std::accumulate(this->cpu_times.begin(), this->cpu_times.end(), 0.0) / this->cpu_times.size();

            // Average sensor rates
            double avg_imu_rate = (this->imu_rates.size() > 0) ?
                std::accumulate(this->imu_rates.begin(), this->imu_rates.end(), 0.0) / this->imu_rates.size() : 0.0;
            double avg_lidar_rate = (this->lidar_rates.size() > 0) ?
                std::accumulate(this->lidar_rates.begin(), this->lidar_rates.end(), 0.0) / this->lidar_rates.size() : 0.0;

            // RAM Usage
            double resident_set = 0.0;
            std::ifstream stat_stream("/proc/self/stat", std::ios_base::in); //get info from proc directory
            std::string pid, comm, state, ppid, pgrp, session, tty_nr;
            std::string tpgid, flags, minflt, cminflt, majflt, cmajflt;
            std::string utime, stime, cutime, cstime, priority, nice;
            std::string num_threads, itrealvalue, starttime;
            unsigned long vsize;
            long rss;
            stat_stream >> pid >> comm >> state >> ppid >> pgrp >> session >> tty_nr
                        >> tpgid >> flags >> minflt >> cminflt >> majflt >> cmajflt
                        >> utime >> stime >> cutime >> cstime >> priority >> nice
                        >> num_threads >> itrealvalue >> starttime >> vsize >> rss; // don't care about the rest
            stat_stream.close();
            long page_size_kb = sysconf(_SC_PAGE_SIZE) / 1024; // for x86-64 is configured to use 2MB pages
            resident_set = rss * page_size_kb;

            // CPU Usage
            struct tms timeSample;
            clock_t now;
            double cpu_percent;
            now = times(&timeSample);
            if (now <= this->lastCPU || timeSample.tms_stime < this->lastSysCPU ||
                timeSample.tms_utime < this->lastUserCPU) {
                cpu_percent = -1.0;
            } else {
                cpu_percent = (timeSample.tms_stime - this->lastSysCPU) + (timeSample.tms_utime - this->lastUserCPU);
                cpu_percent /= (now - this->lastCPU);
                cpu_percent /= this->numProcessors;
                cpu_percent *= 100.;
            }
            this->lastCPU = now;
            this->lastSysCPU = timeSample.tms_stime;
            this->lastUserCPU = timeSample.tms_utime;
            this->cpu_percents.push_front(cpu_percent);
            double avg_cpu_usage =
                std::accumulate(this->cpu_percents.begin(), this->cpu_percents.end(), 0.0) / this->cpu_percents.size();

            // ------------------------------------- PRINT OUT -------------------------------------

            if(this->config.verbose){

            printf("\033[2J\033[1;1H");
            std::cout << std::endl
                        << "+-------------------------------------------------------------------+" << std::endl;
            std::cout   << "|                          LIO ROS (LieOdyssey)                     |" << std::endl;
            std::cout   << "+-------------------------------------------------------------------+" << std::endl;

            std::time_t curr_time = this->scan_stamp;
            std::string asc_time = std::asctime(std::localtime(&curr_time)); asc_time.pop_back();
            std::cout << "| " << std::left << asc_time;
            std::cout << std::right << std::setfill(' ') << std::setw(42)
                << "Elapsed Time: " + to_string_with_precision(this->imu_stamp - this->first_imu_stamp, 2) + " seconds "
                << "|" << std::endl;

            if ( !this->cpu_type.empty() ) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << this->cpu_type + " x " + std::to_string(this->numProcessors)
                << "|" << std::endl;
            }

            if (this->sensor == lidar_odometry_ros::SensorType::OUSTER) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Ouster @ " + to_string_with_precision(avg_lidar_rate, 2)
                                            + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            } else if (this->sensor == lidar_odometry_ros::SensorType::VELODYNE) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Velodyne @ " + to_string_with_precision(avg_lidar_rate, 2)
                                                + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            } else if (this->sensor == lidar_odometry_ros::SensorType::HESAI) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Hesai @ " + to_string_with_precision(avg_lidar_rate, 2)
                                            + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            } else if (this->sensor == lidar_odometry_ros::SensorType::LIVOX) {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Livox @ " + to_string_with_precision(avg_lidar_rate, 2)
                                            + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            } else {
                std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Sensor Rates: Unknown LiDAR @ " + to_string_with_precision(avg_lidar_rate, 2)
                                                    + " Hz, IMU @ " + to_string_with_precision(avg_imu_rate, 2) + " Hz"
                << "|" << std::endl;
            }

            std::cout << "|===================================================================|" << std::endl;

            State final_state = this->getWorldState();

            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Position     {W}  [xyz] :: " + to_string_with_precision(final_state.p(0), 4) + " "
                                            + to_string_with_precision(final_state.p(1), 4) + " "
                                            + to_string_with_precision(final_state.p(2), 4)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Orientation  {W} [wxyz] :: " + to_string_with_precision(final_state.q.w(), 4) + " "
                                            + to_string_with_precision(final_state.q.x(), 4) + " "
                                            + to_string_with_precision(final_state.q.y(), 4) + " "
                                            + to_string_with_precision(final_state.q.z(), 4)
                << "|" << std::endl;

            auto euler = final_state.q.toRotationMatrix().eulerAngles(2, 1, 0);
            double yaw = euler[0] * (180.0/M_PI);
            double pitch = euler[1] * (180.0/M_PI);
            double roll = euler[2] * (180.0/M_PI);

            // use alternate representation if the yaw is smaller
            if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
                yaw   = remainder(yaw + 180.0,   360.0);
                pitch = remainder(180.0 - pitch, 360.0);
                roll  = remainder(roll + 180.0,  360.0);
            }
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "             {W} [ypr] :: " + to_string_with_precision(yaw, 4) + " "
                                            + to_string_with_precision(pitch, 4) + " "
                                            + to_string_with_precision(roll, 4)
                << "|" << std::endl;

            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Lin Velocity {B}  [xyz] :: " + to_string_with_precision(final_state.v(0), 4) + " "
                                            + to_string_with_precision(final_state.v(1), 4) + " "
                                            + to_string_with_precision(final_state.v(2), 4)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Ang Velocity {B}  [xyz] :: " + to_string_with_precision(final_state.w(0), 4) + " "
                                            + to_string_with_precision(final_state.w(1), 4) + " "
                                            + to_string_with_precision(final_state.w(2), 4)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Accel Bias        [xyz] :: " + to_string_with_precision(final_state.b.accel(0), 8) + " "
                                            + to_string_with_precision(final_state.b.accel(1), 8) + " "
                                            + to_string_with_precision(final_state.b.accel(2), 8)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Gyro Bias         [xyz] :: " + to_string_with_precision(final_state.b.gyro(0), 8) + " "
                                            + to_string_with_precision(final_state.b.gyro(1), 8) + " "
                                            + to_string_with_precision(final_state.b.gyro(2), 8)
                << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Gravity Est.      [xyz] :: " + to_string_with_precision(final_state.g(0), 8) + " "
                                            + to_string_with_precision(final_state.g(1), 8) + " "
                                            + to_string_with_precision(final_state.g(2), 8)
                << "|" << std::endl;

            std::cout << "|                                                                   |" << std::endl;


            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "LiDAR -> BaseLink     [t] :: " + to_string_with_precision(this->extr.lidar2baselink.t(0), 4) + " "
                                            + to_string_with_precision(this->extr.lidar2baselink.t(1), 4) + " "
                                            + to_string_with_precision(this->extr.lidar2baselink.t(2), 4)
                << "|" << std::endl;
            
            euler = this->extr.lidar2baselink.R.eulerAngles(2, 1, 0);
            yaw = euler[0] * (180.0/M_PI);
            pitch = euler[1] * (180.0/M_PI);
            roll = euler[2] * (180.0/M_PI);

            // use alternate representation if the yaw is smaller
            if (abs(remainder(yaw + 180.0, 360.0)) < abs(yaw)) {
                yaw   = remainder(yaw + 180.0,   360.0);
                pitch = remainder(180.0 - pitch, 360.0);
                roll  = remainder(roll + 180.0,  360.0);
            }
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "                      [ypr] :: " + to_string_with_precision(yaw, 4) + " "
                                            + to_string_with_precision(pitch, 4) + " "
                                            + to_string_with_precision(roll, 4)
                << "|" << std::endl;

            std::cout << "|                                                                   |" << std::endl;

            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Deskewed points: " + std::to_string(this->deskew_size) << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Integrated states: " + std::to_string(this->propagated_size) << "|" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "Number of matches: " + std::to_string(this->num_matches) << "|" << std::endl;
            std::cout << "|                                                                   |" << std::endl;

            std::cout << std::right << std::setprecision(2) << std::fixed;
            std::cout << "| Computation Time :: "
                << std::setfill(' ') << std::setw(6) << this->cpu_times.front()*1000. << " ms    // Avg: "
                << std::setw(6) << avg_comp_time*1000. << " / Max: "
                << std::setw(6) << *std::max_element(this->cpu_times.begin(), this->cpu_times.end())*1000.
                << "     |" << std::endl;
            std::cout << "| Cores Utilized   :: "
                << std::setfill(' ') << std::setw(6) << (cpu_percent/100.) * this->numProcessors << " cores // Avg: "
                << std::setw(6) << (avg_cpu_usage/100.) * this->numProcessors << " / Max: "
                << std::setw(6) << (*std::max_element(this->cpu_percents.begin(), this->cpu_percents.end()) / 100.)
                                * this->numProcessors
                << "     |" << std::endl;
            std::cout << "| CPU Load         :: "
                << std::setfill(' ') << std::setw(6) << cpu_percent << " %     // Avg: "
                << std::setw(6) << avg_cpu_usage << " / Max: "
                << std::setw(6) << *std::max_element(this->cpu_percents.begin(), this->cpu_percents.end())
                << "     |" << std::endl;
            std::cout << "| " << std::left << std::setfill(' ') << std::setw(66)
                << "RAM Allocation   :: " + to_string_with_precision(resident_set/1000., 2) + " MB"
                << "|" << std::endl;

            std::cout << "+-------------------------------------------------------------------+" << std::endl;

            }//end if(config.verbose)

            // Overwrite CPU stats
            this->mtx_cpu_stats.lock();
            this->cpu_time = cpu_times.front()*1000.0f;
            this->cpu_mean_time = avg_comp_time*1000.0f;
            this->cpu_max_time = *std::max_element(this->cpu_times.begin(), this->cpu_times.end())*1000.0f;
            this->cpu_cores = (cpu_percent/100.0f) * this->numProcessors;
            this->cpu_load = cpu_percent;
            this->cpu_max_load = *std::max_element(this->cpu_percents.begin(), this->cpu_percents.end());
            this->ram_usage = resident_set/1000.0f;
            this->mtx_cpu_stats.unlock();

        }
