#include <signal.h>

// LiDAR Odometry Library
#include "lidar_odometry_ros/common.hpp"
#include "lidar_odometry_ros/modules/localizer.hpp"
#include "lidar_odometry_ros/modules/mapper.hpp"

// ROS
#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>

#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <pcl_conversions/pcl_conversions.h>


class LIOwrapper : public rclcpp::Node
{

    // VARIABLES

    public:
        std::string world_frame;
        std::string body_frame;

        bool publish_tf;

    private:
            // subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr         imu_sub_;

            // main publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr       state_pub;

            // debug publishers
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr orig_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr desk_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr match_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr finalraw_pub;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr body_pub;
        rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr map_bb_pub;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr match_points_pub;

            // TF 
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // FUNCTIONS

    public:

        LIOwrapper() : Node("lidar_odometry_ros", 
                            rclcpp::NodeOptions()
                            .automatically_declare_parameters_from_overrides(true) ) 
            {

                // Declare the one and only Localizer object (singleton pattern)
                lidar_odometry_ros::Localizer& LOC = lidar_odometry_ros::Localizer::getInstance();

                // Load config
                lidar_odometry_ros::Config config;
                this->loadConfig(&config);

                rclcpp::Parameter tf_pub = this->get_parameter("frames.tf_pub");
                this->publish_tf = tf_pub.as_bool();

                // Define two callback groups (ensure parallel execution of lidar_callback & imu_callback)
                rclcpp::SubscriptionOptions lidar_opt, imu_opt;
                lidar_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
                imu_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

                // Set up subscribers
                lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                                config.topics.lidar, 1, std::bind(&LIOwrapper::lidar_callback, this, std::placeholders::_1), lidar_opt);
                imu_sub_   = this->create_subscription<sensor_msgs::msg::Imu>(
                                config.topics.imu, 1000, std::bind(&LIOwrapper::imu_callback, this, std::placeholders::_1), imu_opt);
                
                // Set up publishers
                pc_pub      = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_odometry_ros/pointcloud", 1);
                state_pub   = this->create_publisher<nav_msgs::msg::Odometry>("/lidar_odometry_ros/state", 1);

                orig_pub     = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_odometry_ros/original", 1);
                desk_pub     = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_odometry_ros/deskewed", 1);
                match_pub    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_odometry_ros/match", 1);
                finalraw_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lidar_odometry_ros/final_raw", 1);
                body_pub     = this->create_publisher<nav_msgs::msg::Odometry>("/lidar_odometry_ros/body", 1);
                match_points_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lidar_odometry_ros/match_points", 1);

                // Init TF broadcaster
                tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

                // Initialize Localizer
                LOC.init(config);
            }
        
        private:
        
        /* ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
            ///////////////////////////////////////             Callbacks            ///////////////////////////////////////////////////////////// 
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

        void lidar_callback(const sensor_msgs::msg::PointCloud2 & msg) {
            
            lidar_odometry_ros::Localizer& loc = lidar_odometry_ros::Localizer::getInstance();
            static bool pc_in_good_shape = this->checkPointcloudStructure(msg, loc.get_sensor_type());

            if(not pc_in_good_shape){
                throw std::runtime_error("lidar_odometry_ros::FATAL ERROR: invalid pointcloud structure\n\n");
            }

            pcl::PointCloud<PointType>::Ptr pc_ (std::make_shared<pcl::PointCloud<PointType>>());
            pcl::fromROSMsg(msg, *pc_);

            loc.updatePointCloud(pc_, rclcpp::Time(msg.header.stamp).seconds());

            // Publish output pointcloud
            sensor_msgs::msg::PointCloud2 pc_ros;
            pcl::toROSMsg(*loc.get_pointcloud(), pc_ros);
            pc_ros.header.stamp = this->get_clock()->now();
            pc_ros.header.frame_id = this->world_frame;
            this->pc_pub->publish(pc_ros);

            // Publish debugging pointclouds
            sensor_msgs::msg::PointCloud2 orig_msg;
            pcl::toROSMsg(*loc.get_orig_pointcloud(), orig_msg);
            orig_msg.header.stamp = this->get_clock()->now();
            orig_msg.header.frame_id = this->body_frame;
            this->orig_pub->publish(orig_msg);

            sensor_msgs::msg::PointCloud2 deskewed_msg;
            pcl::toROSMsg(*loc.get_deskewed_pointcloud(), deskewed_msg);
            deskewed_msg.header.stamp = this->get_clock()->now();
            deskewed_msg.header.frame_id = this->world_frame;
            this->desk_pub->publish(deskewed_msg);

            sensor_msgs::msg::PointCloud2 match_msg;
            pcl::toROSMsg(*loc.get_pc2match_pointcloud(), match_msg);
            match_msg.header.stamp = this->get_clock()->now();
            match_msg.header.frame_id = this->body_frame;
            this->match_pub->publish(match_msg);

            sensor_msgs::msg::PointCloud2 finalraw_msg;
            pcl::toROSMsg(*loc.get_finalraw_pointcloud(), finalraw_msg);
            finalraw_msg.header.stamp = this->get_clock()->now();
            finalraw_msg.header.frame_id = this->world_frame;
            this->finalraw_pub->publish(finalraw_msg);

            // Visualize current matches
            visualization_msgs::msg::MarkerArray match_markers = this->getMatchesMarker(loc.get_matches(), 
                                                                                    this->world_frame
                                                                                    );
            this->match_points_pub->publish(match_markers);
        }

        void imu_callback(const sensor_msgs::msg::Imu & msg) {

            lidar_odometry_ros::Localizer& loc = lidar_odometry_ros::Localizer::getInstance();

            lidar_odometry_ros::IMUmeas imu;
            this->fromROStoLimo(msg, imu);

            // Propagate IMU measurement
            loc.updateIMU(imu);

            // State publishing
            nav_msgs::msg::Odometry state_msg, body_msg;
            this->fromLimoToROS(loc.getWorldState(), loc.getPoseCovariance(), loc.getTwistCovariance(), state_msg);
            this->fromLimoToROS(loc.getBodyState(), loc.getPoseCovariance(), loc.getTwistCovariance(), body_msg);

            this->state_pub->publish(state_msg);
            this->body_pub->publish(body_msg);

            // TF broadcasting
            if(this->publish_tf)
                this->broadcastTF(loc.getWorldState(), world_frame, body_frame, true);
        }

    /* ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
        ///////////////////////////////////////             Load params          ///////////////////////////////////////////////////////////// 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

        void loadConfig(lidar_odometry_ros::Config* config){

            // Topics
            rclcpp::Parameter lidar_topic_p = this->get_parameter("topics.input.lidar");
            rclcpp::Parameter imu_topic_p   = this->get_parameter("topics.input.imu");
            config->topics.lidar = lidar_topic_p.as_string();
            config->topics.imu = imu_topic_p.as_string();

            // Frames
            rclcpp::Parameter world_p = this->get_parameter("frames.world");
            rclcpp::Parameter body_p = this->get_parameter("frames.body");
            this->world_frame = world_p.as_string();
            this->body_frame = body_p.as_string();

            // General
            rclcpp::Parameter n_thread_p = this->get_parameter("num_threads");
            config->num_threads = n_thread_p.as_int();
            rclcpp::Parameter sensor_p = this->get_parameter("sensor_type");
            config->sensor_type = sensor_p.as_int();
            rclcpp::Parameter debug_p = this->get_parameter("debug");
            config->debug = debug_p.as_bool();
            rclcpp::Parameter verbose_p = this->get_parameter("verbose");
            config->verbose = verbose_p.as_bool();
            rclcpp::Parameter offset_p = this->get_parameter("time_offset");
            config->time_offset = offset_p.as_bool();
            rclcpp::Parameter eos_p = this->get_parameter("end_of_sweep");
            config->end_of_sweep = eos_p.as_bool();
            rclcpp::Parameter motion_comp_p = this->get_parameter("motion_compensation");
            config->motion_compensation = motion_comp_p.as_bool();

            // Calibration
            rclcpp::Parameter grav_p = this->get_parameter("calibration.gravity_align");
            config->gravity_align = grav_p.as_bool();
            rclcpp::Parameter est_accel_p = this->get_parameter("calibration.accel");
            config->calibrate_accel = est_accel_p.as_bool();
            rclcpp::Parameter est_gyro_p = this->get_parameter("calibration.gyro");
            config->calibrate_gyro = est_gyro_p.as_bool();
            rclcpp::Parameter est_time_p = this->get_parameter("calibration.time");
            config->imu_calib_time = est_time_p.as_double();

            // Extrinsics
            rclcpp::Parameter extr_imu_t_p = this->get_parameter("extrinsics.imu.t");
            std::vector<double> imu2baselink_t = extr_imu_t_p.as_double_array();
            config->extrinsics.imu2baselink_t = std::vector<float>(imu2baselink_t.begin(), imu2baselink_t.end());
            rclcpp::Parameter extr_imu_R_p = this->get_parameter("extrinsics.imu.R");
            std::vector<double> imu2baselink_R = extr_imu_R_p.as_double_array();
            config->extrinsics.imu2baselink_R = std::vector<float>(imu2baselink_R.begin(), imu2baselink_R.end());
            rclcpp::Parameter extr_lidar_t_p = this->get_parameter("extrinsics.lidar.t");
            std::vector<double> lidar2baselink_t = extr_lidar_t_p.as_double_array();
            config->extrinsics.lidar2baselink_t = std::vector<float>(lidar2baselink_t.begin(), lidar2baselink_t.end());
            rclcpp::Parameter extr_lidar_R_p = this->get_parameter("extrinsics.lidar.R");
            std::vector<double> lidar2baselink_R = extr_lidar_R_p.as_double_array();
            config->extrinsics.lidar2baselink_R = std::vector<float>(lidar2baselink_R.begin(), lidar2baselink_R.end());

            // Intrinsics
            rclcpp::Parameter intr_accel_bias_p = this->get_parameter("intrinsics.accel.bias");
            std::vector<double> accel_bias = intr_accel_bias_p.as_double_array();
            config->intrinsics.accel_bias = std::vector<float>(accel_bias.begin(), accel_bias.end());
            rclcpp::Parameter intr_accel_sm_p = this->get_parameter("intrinsics.accel.sm");
            std::vector<double> imu_sm = intr_accel_sm_p.as_double_array();
            config->intrinsics.imu_sm = std::vector<float>(imu_sm.begin(), imu_sm.end());
            rclcpp::Parameter intr_gyro_bias_p = this->get_parameter("intrinsics.gyro.bias");
            std::vector<double> gyro_bias = intr_gyro_bias_p.as_double_array();
            config->intrinsics.gyro_bias = std::vector<float>(gyro_bias.begin(), gyro_bias.end());

            // Crop Box filter
            rclcpp::Parameter filt_crop_flag_p = this->get_parameter("filters.cropBox.active");
            config->filters.crop_active = filt_crop_flag_p.as_bool();
            rclcpp::Parameter filt_crop_min_p = this->get_parameter("filters.cropBox.box.min");
            std::vector<double> cropBoxMin = filt_crop_min_p.as_double_array();
            config->filters.cropBoxMin = std::vector<float>(cropBoxMin.begin(), cropBoxMin.end());
            rclcpp::Parameter filt_crop_max_p = this->get_parameter("filters.cropBox.box.max");
            std::vector<double> cropBoxMax = filt_crop_max_p.as_double_array();
            config->filters.cropBoxMax = std::vector<float>(cropBoxMax.begin(), cropBoxMax.end());

            // Voxel Grid filter
            rclcpp::Parameter filt_voxel_flag_p = this->get_parameter("filters.voxelGrid.active");
            config->filters.voxel_active = filt_voxel_flag_p.as_bool();
            rclcpp::Parameter filt_voxel_size_p = this->get_parameter("filters.voxelGrid.leafSize");
            std::vector<double> leafSize = filt_voxel_size_p.as_double_array();
            config->filters.leafSize = std::vector<float>(leafSize.begin(), leafSize.end());

            // Sphere crop filter
            rclcpp::Parameter filt_dist_flag_p = this->get_parameter("filters.minDistance.active");
            config->filters.dist_active = filt_dist_flag_p.as_bool();
            rclcpp::Parameter filt_dist_val_p = this->get_parameter("filters.minDistance.value");
            config->filters.min_dist = filt_dist_val_p.as_double();

            // FoV filter
            rclcpp::Parameter filt_fov_flag_p = this->get_parameter("filters.FoV.active");
            config->filters.fov_active = filt_fov_flag_p.as_bool();
            rclcpp::Parameter filt_fov_val_p = this->get_parameter("filters.FoV.value");
            config->filters.fov_angle = static_cast<float>(filt_fov_val_p.as_double()*M_PI/360.0); // half of FoV (bc. is divided by the x-axis)

            // Sampling Rate filter
            rclcpp::Parameter filt_rate_flag_p = this->get_parameter("filters.rateSampling.active");
            config->filters.rate_active = filt_rate_flag_p.as_bool();
            rclcpp::Parameter filt_rate_val_p = this->get_parameter("filters.rateSampling.value");
            config->filters.rate_value = filt_rate_val_p.as_int();

            // iESEKF config
            rclcpp::Parameter max_iters_p = this->get_parameter("iESEKF.MAX_NUM_ITERS");
            config->ekf.MAX_NUM_ITERS = max_iters_p.as_int();
            rclcpp::Parameter max_match_p = this->get_parameter("iESEKF.MAX_NUM_MATCHES");
            config->mapping.MAX_NUM_MATCHES = max_match_p.as_int();
            rclcpp::Parameter max_pc_p = this->get_parameter("iESEKF.MAX_NUM_PC2MATCH");
            config->mapping.MAX_NUM_PC2MATCH = max_pc_p.as_int();
            rclcpp::Parameter limits_p = this->get_parameter("iESEKF.TOLERANCE");
            config->ekf.TOLERANCE = limits_p.as_double();
            rclcpp::Parameter lidar_n_p = this->get_parameter("iESEKF.LIDAR_NOISE");
            config->mapping.LIDAR_NOISE = lidar_n_p.as_double();

            // Mapping
            rclcpp::Parameter match_point_p = this->get_parameter("iESEKF.Mapping.NUM_MATCH_POINTS");
            config->mapping.NUM_MATCH_POINTS = match_point_p.as_int();
            rclcpp::Parameter max_plane_dist_p = this->get_parameter("iESEKF.Mapping.MAX_DIST_PLANE");
            config->mapping.MAX_DIST_PLANE = max_plane_dist_p.as_double();
            rclcpp::Parameter plane_thr_p = this->get_parameter("iESEKF.Mapping.PLANES_THRESHOLD");
            config->mapping.PLANE_THRESHOLD = plane_thr_p.as_double();

            // iOcTree 
            rclcpp::Parameter bucket_size = this->get_parameter("iESEKF.Mapping.Octree.bucket_size");
            config->mapping.octree.bucket_size = bucket_size.as_int();
            rclcpp::Parameter min_extent = this->get_parameter("iESEKF.Mapping.Octree.min_extent");
            config->mapping.octree.min_extent = static_cast<float>(min_extent.as_double());

            // Covariance
            rclcpp::Parameter gyro_p = this->get_parameter("iESEKF.covariance.gyro");
            config->ekf.cov_gyro = gyro_p.as_double();
            rclcpp::Parameter accel_p = this->get_parameter("iESEKF.covariance.accel");
            config->ekf.cov_acc = accel_p.as_double();
            rclcpp::Parameter gyro_bias_p = this->get_parameter("iESEKF.covariance.bias_gyro");
            config->ekf.cov_bias_gyro = gyro_bias_p.as_double();
            rclcpp::Parameter accel_bias_p = this->get_parameter("iESEKF.covariance.bias_accel");
            config->ekf.cov_bias_acc = accel_bias_p.as_double();
        }

    
    /* ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
        ///////////////////////////////////////             Aux. func.           ///////////////////////////////////////////////////////////// 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

        void fromROStoLimo(const sensor_msgs::msg::Imu& in, lidar_odometry_ros::IMUmeas& out){
            out.stamp = rclcpp::Time(in.header.stamp).seconds();

            out.ang_vel(0) = in.angular_velocity.x;
            out.ang_vel(1) = in.angular_velocity.y;
            out.ang_vel(2) = in.angular_velocity.z;

            out.lin_accel(0) = in.linear_acceleration.x;
            out.lin_accel(1) = in.linear_acceleration.y;
            out.lin_accel(2) = in.linear_acceleration.z;

            Eigen::Quaterniond qd(in.orientation.w, 
                                in.orientation.x, 
                                in.orientation.y, 
                                in.orientation.z );
            out.q = qd.cast<float>();
        }

        void fromLimoToROS(const lidar_odometry_ros::State& in, nav_msgs::msg::Odometry& out){
            out.header.stamp = this->get_clock()->now();
            out.header.frame_id = "map";

            // Pose/Attitude
            Eigen::Vector3d pos = in.p.cast<double>();
            out.pose.pose.position.x = pos(0);
            out.pose.pose.position.y = pos(1);
            out.pose.pose.position.z = pos(2);

            Eigen::Quaterniond quat = in.q.cast<double>();
            out.pose.pose.orientation.x = quat.x();
            out.pose.pose.orientation.y = quat.y();
            out.pose.pose.orientation.z = quat.z();
            out.pose.pose.orientation.w = quat.w();

            // Twist
            Eigen::Vector3d lin_v = in.v.cast<double>();
            out.twist.twist.linear.x  = lin_v(0);
            out.twist.twist.linear.y  = lin_v(1);
            out.twist.twist.linear.z  = lin_v(2);

            Eigen::Vector3d ang_v = in.w.cast<double>();
            out.twist.twist.angular.x = ang_v(0);
            out.twist.twist.angular.y = ang_v(1);
            out.twist.twist.angular.z = ang_v(2);
        }

        void fromLimoToROS(const lidar_odometry_ros::State& in, const std::vector<double>& cov_pose,
                            const std::vector<double>& cov_twist, nav_msgs::msg::Odometry& out){

            this->fromLimoToROS(in, out);

            // Covariances
            for(long unsigned int i=0; i<cov_pose.size(); i++){
                out.pose.covariance[i]  = cov_pose[i];
                out.twist.covariance[i] = cov_twist[i];
            }
        }

        void broadcastTF(const lidar_odometry_ros::State& in, std::string parent_name, std::string child_name, bool now){

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp    = (now) ? this->get_clock()->now() : rclcpp::Time(in.time);
            /* NOTE: depending on IMU sensor rate, the state's stamp could be too old, 
                so a TF warning could be print out (really annoying!).
                In order to avoid this, the "now" argument should be true.
            */
            tf_msg.header.frame_id = parent_name;
            tf_msg.child_frame_id  = child_name;

            // Translation
            Eigen::Vector3d pos = in.p.cast<double>();
            tf_msg.transform.translation.x = pos(0);
            tf_msg.transform.translation.y = pos(1);
            tf_msg.transform.translation.z = pos(2);

            // Rotation
            Eigen::Quaterniond quat = in.q.cast<double>();
            tf_msg.transform.rotation.x = quat.x();
            tf_msg.transform.rotation.y = quat.y();
            tf_msg.transform.rotation.z = quat.z();
            tf_msg.transform.rotation.w = quat.w();

            // Broadcast
            tf_broadcaster_->sendTransform(tf_msg);
        }

        bool checkPointcloudStructure(const sensor_msgs::msg::PointCloud2 & msg, lidar_odometry_ros::SensorType sensor){

            using sensor_msgs::msg::PointField;

            if (sensor == lidar_odometry_ros::SensorType::OUSTER) {
                for(size_t i=0; i < msg.fields.size(); i++){
                    if( (msg.fields[i].name == "t") && (msg.fields[i].datatype == PointField::UINT32) )
                        return true;
                }
        
                RCLCPP_ERROR_STREAM(this->get_logger(), "\n-------------------------------------------------------------------\n" 
                                    << "lidar_odometry_ros::FATAL ERROR: the received pointcloud MUST have a timestamp field available!\n"
                                    << "          Remember that for OUSTER alike pointclouds, the expected fields are:\n"
                                    << "                  x: FLOAT32 (x coordinate in meters)\n"
                                    << "                  y: FLOAT32 (y coordinate in meters)\n"
                                    << "                  z: FLOAT32 (z coordinate in meters)\n"
                                    << "                  t: UINT32 (time since beginning of scan in nanoseconds)\n"
                                    << "-------------------------------------------------------------------\n"
                                    );
        
            } else if (sensor == lidar_odometry_ros::SensorType::VELODYNE) {
                for(size_t i=0; i < msg.fields.size(); i++){
                    if( (msg.fields[i].name == "time") && (msg.fields[i].datatype == PointField::FLOAT32)  )
                        return true;
                }

                RCLCPP_ERROR_STREAM(this->get_logger(), "\n-------------------------------------------------------------------\n" 
                                    << "lidar_odometry_ros::FATAL ERROR: the received pointcloud MUST have a timestamp field available!\n"
                                    << "          Remember that for VELODYNE alike pointclouds, the expected fields are:\n"
                                    << "                  x: FLOAT32 (x coordinate in meters)\n"
                                    << "                  y: FLOAT32 (y coordinate in meters)\n"
                                    << "                  z: FLOAT32 (z coordinate in meters)\n"
                                    << "                  time: FLOAT32 (time since beginning of scan in seconds)\n"
                                    << "-------------------------------------------------------------------\n"
                                    );
        
            } else if ( (sensor == lidar_odometry_ros::SensorType::HESAI) || (sensor == lidar_odometry_ros::SensorType::LIVOX) ) {
                for(size_t i=0; i < msg.fields.size(); i++){
                    if( (msg.fields[i].name == "timestamp") && (msg.fields[i].datatype == PointField::FLOAT64) )
                        return true;
                }

                RCLCPP_ERROR_STREAM(this->get_logger(), "\n-------------------------------------------------------------------\n" 
                                    << "lidar_odometry_ros::FATAL ERROR: the received pointcloud MUST have a timestamp field available!\n"
                                    << "          Remember that for HESAI/LIVOX alike pointclouds, the expected fields are:\n"
                                    << "                  x: FLOAT32 (x coordinate in meters)\n"
                                    << "                  y: FLOAT32 (y coordinate in meters)\n"
                                    << "                  z: FLOAT32 (z coordinate in meters)\n"
                                    << "                  timestamp: FLOAT64 (global time in seconds/nanoseconds if HESAI/LIVOX)\n"
                                    << "-------------------------------------------------------------------\n"
                                    );
        
            } else {
                RCLCPP_ERROR_STREAM(this->get_logger(), "\n-------------------------------------------------------------------\n" 
                                    << "lidar_odometry_ros::FATAL ERROR: LiDAR sensor type unknown or not specified!\n"
                                    << "-------------------------------------------------------------------\n"
                                    );
            }
            
            return false;
        }

        visualization_msgs::msg::MarkerArray getMatchesMarker(Matches& matches, std::string frame_id){
            visualization_msgs::msg::MarkerArray m_array;
            visualization_msgs::msg::Marker m;

            m_array.markers.reserve(matches.size());

            m.ns = "lidar_odometry_ros_match";
            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;

            m.color.r = 0.0f;
            m.color.g = 0.0f;
            m.color.b = 1.0f;
            m.color.a = 1.0f;

            m.lifetime = rclcpp::Duration::from_seconds(0.0);
            m.header.frame_id = frame_id;
            m.header.stamp = this->get_clock()->now();

            m.pose.orientation.w = 1.0;

            m.scale.x = 0.2;
            m.scale.y = 0.2;
            m.scale.z = 0.2;

            for(std::size_t i=0; i < matches.size(); i++){
                m.id = i;
                Eigen::Vector3f match_p = matches[i].get_global_point();
                m.pose.position.x = match_p(0);
                m.pose.position.y = match_p(1);
                m.pose.position.z = match_p(2);

                m_array.markers.push_back(m);
            }

            return m_array;
        }

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::Node::SharedPtr lio_node = std::make_shared<LIOwrapper>();

    rclcpp::executors::MultiThreadedExecutor executor; // by default using all available cores
    executor.add_node(lio_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}