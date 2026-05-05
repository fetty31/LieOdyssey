#include <signal.h>

// LiDAR Odometry Library
#include "lio_ros/odometry_core.hpp"

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

        LIOwrapper() : Node("lio_ros_node", 
                            rclcpp::NodeOptions()
                            .automatically_declare_parameters_from_overrides(true) ) 
            { }

        void init()
        {
            // Declare the one and only Core object (singleton pattern)
            lio_ros::OdometryCore& core = lio_ros::OdometryCore::getInstance();

            // Load config
            lio_ros::Config config;
            this->loadConfig(&config);

            rclcpp::Parameter tf_pub = this->get_parameter("frames.tf_pub");
            this->publish_tf = tf_pub.as_bool();

            // Define two callback groups (ensure parallel execution of lidar_callback & imu_callback)
            rclcpp::SubscriptionOptions lidar_opt, imu_opt;
            lidar_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            imu_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

            // Set up subscribers
            lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                            config.lidar_topic, 1, std::bind(&LIOwrapper::lidar_callback, this, std::placeholders::_1), lidar_opt);
            imu_sub_   = this->create_subscription<sensor_msgs::msg::Imu>(
                            config.imu_topic, 1000, std::bind(&LIOwrapper::imu_callback, this, std::placeholders::_1), imu_opt);
            
            // Set up publishers
            pc_pub      = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lio_ros/pointcloud", 1);
            state_pub   = this->create_publisher<nav_msgs::msg::Odometry>("/lio_ros/state", 1);

            desk_pub     = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lio_ros/deskewed", 1);
            match_pub    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lio_ros/match", 1);
            finalraw_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lio_ros/pointcloud_raw", 1);
            body_pub     = this->create_publisher<nav_msgs::msg::Odometry>("/lio_ros/body", 1);
            match_points_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/lio_ros/match_points", 1);

            // Init TF broadcaster
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

            // Initialize LIO core
            core.initialize(config);
        }
        
        private:
        
        /* ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
            ///////////////////////////////////////             Callbacks            ///////////////////////////////////////////////////////////// 
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

        void lidar_callback(const sensor_msgs::msg::PointCloud2 & msg) {
            
            lio_ros::OdometryCore& core = lio_ros::OdometryCore::getInstance();
            static bool pc_in_good_shape = this->checkPointcloudStructure(msg, core.get_sensor_type());

            if(not pc_in_good_shape){
                throw std::runtime_error("lio_ros::FATAL ERROR: invalid pointcloud structure\n\n");
            }

            pcl::PointCloud<LioPointType>::Ptr pc_ (std::make_shared<pcl::PointCloud<LioPointType>>());
            pcl::fromROSMsg(msg, *pc_);

            core.processScan(pc_, rclcpp::Time(msg.header.stamp).seconds());

            // Publish output pointcloud
            sensor_msgs::msg::PointCloud2 pc_ros;
            pcl::toROSMsg(*core.getWorldScan(), pc_ros);
            pc_ros.header.stamp = this->get_clock()->now();
            pc_ros.header.frame_id = this->world_frame;
            this->pc_pub->publish(pc_ros);

            // Publish debugging pointclouds
            sensor_msgs::msg::PointCloud2 deskewed_msg;
            pcl::toROSMsg(*core.getDeskewedScan(), deskewed_msg);
            deskewed_msg.header.stamp = this->get_clock()->now();
            deskewed_msg.header.frame_id = this->world_frame;
            this->desk_pub->publish(deskewed_msg);

            sensor_msgs::msg::PointCloud2 match_msg;
            pcl::toROSMsg(*core.getScanToMatch(), match_msg);
            match_msg.header.stamp = this->get_clock()->now();
            match_msg.header.frame_id = this->body_frame;
            this->match_pub->publish(match_msg);

            sensor_msgs::msg::PointCloud2 finalraw_msg;
            pcl::toROSMsg(*core.getRawWorldScan(), finalraw_msg);
            finalraw_msg.header.stamp = this->get_clock()->now();
            finalraw_msg.header.frame_id = this->world_frame;
            this->finalraw_pub->publish(finalraw_msg);

            // Visualize current matches
            // visualization_msgs::msg::MarkerArray match_markers = this->getMatchesMarker(core.get_matches(), 
            //                                                                         this->world_frame
            //                                                                         );
            // this->match_points_pub->publish(match_markers);
        }

        void imu_callback(const sensor_msgs::msg::Imu & msg) {

            lio_ros::OdometryCore& core = lio_ros::OdometryCore::getInstance();

            lie_odyssey::IMUmeas imu;
            this->fromROStoLimo(msg, imu);

            // Propagate IMU measurement
            core.processIMU(imu);

            // State publishing
            nav_msgs::msg::Odometry state_msg, body_msg;
            this->fromLimoToROS(core.getState(),      core.getPoseCovariance(), core.getTwistCovariance(), state_msg);
            this->fromLimoToROS(core.getLiDARState(), core.getPoseCovariance(), core.getTwistCovariance(), body_msg);

            this->state_pub->publish(state_msg);
            this->body_pub->publish(body_msg);

            // TF broadcasting
            if(this->publish_tf)
                this->broadcastTF(core.getState(), world_frame, body_frame, true);
        }

    /* ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
        ///////////////////////////////////////             Load params          ///////////////////////////////////////////////////////////// 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

        void loadConfig(lio_ros::Config* config){

            // Topics
            rclcpp::Parameter lidar_topic_p = this->get_parameter("topics.input.lidar");
            rclcpp::Parameter imu_topic_p   = this->get_parameter("topics.input.imu");
            config->lidar_topic = lidar_topic_p.as_string();
            config->imu_topic   = imu_topic_p.as_string();

            // Frames
            rclcpp::Parameter world_p = this->get_parameter("frames.world");
            rclcpp::Parameter body_p = this->get_parameter("frames.body");
            config->world_frame = world_p.as_string();
            config->body_frame  = body_p.as_string();
            this->world_frame = config->world_frame;
            this->body_frame  = config->body_frame;

            // General
            rclcpp::Parameter n_thread_p = this->get_parameter("num_threads");
            config->num_threads = n_thread_p.as_int();

            auto sensor_str = this->get_parameter("sensor_type").as_string();
            if ( (sensor_str == "ouster") || (sensor_str == "OUSTER") ) {
                config->sensor_type = lio_ros::SensorType::OUSTER;
            } else if ( (sensor_str == "velodyne") || (sensor_str == "VELODYNE") ) {
                config->sensor_type = lio_ros::SensorType::VELODYNE;
            } else if ( (sensor_str == "hesai") || (sensor_str == "HESAI") ) {
                config->sensor_type = lio_ros::SensorType::HESAI;
            } else if ( (sensor_str == "livox") || (sensor_str == "LIVOX") ) {
                config->sensor_type = lio_ros::SensorType::LIVOX;
            } else {
                throw std::runtime_error("Invalid sensor_type: " + sensor_str);
            }

            rclcpp::Parameter debug_p = this->get_parameter("debug");
            config->debug = debug_p.as_bool();
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
            auto get_vec3f = [this](const std::string& name) -> Eigen::Vector3f {
                rclcpp::Parameter v_p = this->get_parameter(name);
                std::vector<double> v = v_p.as_double_array();
                if (v.size() != 3) {
                    throw std::runtime_error(name + " must have size 3");
                }
                Eigen::Vector3d vec(v[0], v[1], v[2]);

                if (!vec.allFinite()) {
                    throw std::runtime_error(name + " contains NaN/Inf");
                }
                return vec.cast<float>();
            };
            auto get_mat3f = [this](const std::string& name) -> Eigen::Matrix3f {
                rclcpp::Parameter v_p = this->get_parameter(name);
                std::vector<double> v = v_p.as_double_array();
                if (v.size() != 9) {
                    throw std::runtime_error(name + " must have size 9");
                }
                Eigen::Matrix3d M;
                M << v[0], v[1], v[2],
                    v[3], v[4], v[5],
                    v[6], v[7], v[8];

                if (!M.allFinite()) {
                    throw std::runtime_error(name + " contains NaN/Inf");
                }
                return M.cast<float>();
            };

            Eigen::Vector3f imu_t = get_vec3f("extrinsics.imu.t");
            Eigen::Matrix3f imu_R = get_mat3f("extrinsics.imu.R");

            config->imu_extr.translate(imu_t);
            config->imu_extr.rotate(imu_R.transpose());

            Eigen::Vector3f lidar_t = get_vec3f("extrinsics.lidar.t");
            Eigen::Matrix3f lidar_R = get_mat3f("extrinsics.lidar.R");

            config->lidar_extr.translate(lidar_t);
            config->lidar_extr.rotate(lidar_R.transpose());

            // Intrinsics
            config->accel_bias = get_vec3f("intrinsics.accel.bias");
            config->gyro_bias = get_vec3f("intrinsics.gyro.bias");
            config->imu_axis_matrix = get_mat3f("intrinsics.accel.sm");

            // Crop Box filter
            rclcpp::Parameter filt_crop_flag_p = this->get_parameter("filters.cropBox.active");
            config->crop_active = filt_crop_flag_p.as_bool();
            config->crop_min = get_vec3f("filters.cropBox.box.min");
            config->crop_max = get_vec3f("filters.cropBox.box.max");

            // Voxel Grid filter
            rclcpp::Parameter filt_voxel_flag_p = this->get_parameter("filters.voxelGrid.active");
            config->voxel_active = filt_voxel_flag_p.as_bool();
            config->voxel_size = get_vec3f("filters.voxelGrid.leafSize");

            // Sphere crop filter
            rclcpp::Parameter filt_dist_flag_p = this->get_parameter("filters.minDistance.active");
            config->dist_active = filt_dist_flag_p.as_bool();
            rclcpp::Parameter filt_dist_val_p = this->get_parameter("filters.minDistance.value");
            config->min_dist = static_cast<float>(filt_dist_val_p.as_double());

            // Sampling Rate filter
            rclcpp::Parameter filt_rate_flag_p = this->get_parameter("filters.rateSampling.active");
            config->rate_active = filt_rate_flag_p.as_bool();
            rclcpp::Parameter filt_rate_val_p = this->get_parameter("filters.rateSampling.value");
            config->rate_value = filt_rate_val_p.as_int();

            // iESEKF config
            rclcpp::Parameter max_iters_p = this->get_parameter("iESEKF.MAX_NUM_ITERS");
            config->max_ekf_iters = max_iters_p.as_int();
            rclcpp::Parameter max_match_p = this->get_parameter("iESEKF.MAX_NUM_MATCHES");
            config->max_num_matches = max_match_p.as_int();
            rclcpp::Parameter max_pc_p = this->get_parameter("iESEKF.MAX_NUM_PC2MATCH");
            config->max_pc2match = max_pc_p.as_int();
            rclcpp::Parameter limits_p = this->get_parameter("iESEKF.TOLERANCE");
            config->ekf_tolerance = limits_p.as_double();
            rclcpp::Parameter lidar_n_p = this->get_parameter("iESEKF.LIDAR_NOISE");
            config->lidar_noise = lidar_n_p.as_double();

            // Mapping
            rclcpp::Parameter match_point_p = this->get_parameter("iESEKF.Mapping.NUM_MATCH_POINTS");
            config->n_points_match = match_point_p.as_int();
            rclcpp::Parameter max_plane_dist_p = this->get_parameter("iESEKF.Mapping.MAX_DIST_PLANE");
            config->max_dist_plane = max_plane_dist_p.as_double();
            rclcpp::Parameter plane_thr_p = this->get_parameter("iESEKF.Mapping.PLANES_THRESHOLD");
            config->plane_threshold = plane_thr_p.as_double();

            // iOcTree 
            rclcpp::Parameter bucket_size = this->get_parameter("iESEKF.Mapping.Octree.bucket_size");
            config->octree_bucket_size = bucket_size.as_int();
            rclcpp::Parameter min_extent = this->get_parameter("iESEKF.Mapping.Octree.min_extent");
            config->octree_extent = static_cast<float>(min_extent.as_double());

            // Covariance
            rclcpp::Parameter gyro_p = this->get_parameter("iESEKF.covariance.gyro");
            config->cov_gyro = gyro_p.as_double();
            rclcpp::Parameter accel_p = this->get_parameter("iESEKF.covariance.accel");
            config->cov_acc = accel_p.as_double();
            rclcpp::Parameter gyro_bias_p = this->get_parameter("iESEKF.covariance.bias_gyro");
            config->cov_bias_gyro = gyro_bias_p.as_double();
            rclcpp::Parameter accel_bias_p = this->get_parameter("iESEKF.covariance.bias_accel");
            config->cov_bias_acc = accel_bias_p.as_double();
        }

    
    /* ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// 
        ///////////////////////////////////////             Aux. func.           ///////////////////////////////////////////////////////////// 
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

        void fromROStoLimo(const sensor_msgs::msg::Imu& in, lie_odyssey::IMUmeas& out){
            out.stamp = rclcpp::Time(in.header.stamp).seconds();

            out.gyro(0) = in.angular_velocity.x;
            out.gyro(1) = in.angular_velocity.y;
            out.gyro(2) = in.angular_velocity.z;

            out.accel(0) = in.linear_acceleration.x;
            out.accel(1) = in.linear_acceleration.y;
            out.accel(2) = in.linear_acceleration.z;
        }

        void fromLimoToROS(const lio_ros::State& in, nav_msgs::msg::Odometry& out){
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

        void fromLimoToROS(const lio_ros::State& in, const std::vector<double>& cov_pose,
                            const std::vector<double>& cov_twist, nav_msgs::msg::Odometry& out){

            this->fromLimoToROS(in, out);

            // Covariances
            for(long unsigned int i=0; i<cov_pose.size(); i++){
                out.pose.covariance[i]  = cov_pose[i];
                out.twist.covariance[i] = cov_twist[i];
            }
        }

        void broadcastTF(const lio_ros::State& in, std::string parent_name, std::string child_name, bool now){

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

        bool checkPointcloudStructure(const sensor_msgs::msg::PointCloud2 & msg, lio_ros::SensorType sensor){

            using sensor_msgs::msg::PointField;

            if (sensor == lio_ros::SensorType::OUSTER) {
                for(size_t i=0; i < msg.fields.size(); i++){
                    if( (msg.fields[i].name == "t") && (msg.fields[i].datatype == PointField::UINT32) )
                        return true;
                }
        
                RCLCPP_ERROR_STREAM(this->get_logger(), "\n-------------------------------------------------------------------\n" 
                                    << "lio_ros::FATAL ERROR: the received pointcloud MUST have a timestamp field available!\n"
                                    << "          Remember that for OUSTER alike pointclouds, the expected fields are:\n"
                                    << "                  x: FLOAT32 (x coordinate in meters)\n"
                                    << "                  y: FLOAT32 (y coordinate in meters)\n"
                                    << "                  z: FLOAT32 (z coordinate in meters)\n"
                                    << "                  t: UINT32 (time since beginning of scan in nanoseconds)\n"
                                    << "-------------------------------------------------------------------\n"
                                    );
        
            } else if (sensor == lio_ros::SensorType::VELODYNE) {
                for(size_t i=0; i < msg.fields.size(); i++){
                    if( (msg.fields[i].name == "time") && (msg.fields[i].datatype == PointField::FLOAT32)  )
                        return true;
                }

                RCLCPP_ERROR_STREAM(this->get_logger(), "\n-------------------------------------------------------------------\n" 
                                    << "lio_ros::FATAL ERROR: the received pointcloud MUST have a timestamp field available!\n"
                                    << "          Remember that for VELODYNE alike pointclouds, the expected fields are:\n"
                                    << "                  x: FLOAT32 (x coordinate in meters)\n"
                                    << "                  y: FLOAT32 (y coordinate in meters)\n"
                                    << "                  z: FLOAT32 (z coordinate in meters)\n"
                                    << "                  time: FLOAT32 (time since beginning of scan in seconds)\n"
                                    << "-------------------------------------------------------------------\n"
                                    );
        
            } else if ( (sensor == lio_ros::SensorType::HESAI) || (sensor == lio_ros::SensorType::LIVOX) ) {
                for(size_t i=0; i < msg.fields.size(); i++){
                    if( (msg.fields[i].name == "timestamp") && (msg.fields[i].datatype == PointField::FLOAT64) )
                        return true;
                }

                RCLCPP_ERROR_STREAM(this->get_logger(), "\n-------------------------------------------------------------------\n" 
                                    << "lio_ros::FATAL ERROR: the received pointcloud MUST have a timestamp field available!\n"
                                    << "          Remember that for HESAI/LIVOX alike pointclouds, the expected fields are:\n"
                                    << "                  x: FLOAT32 (x coordinate in meters)\n"
                                    << "                  y: FLOAT32 (y coordinate in meters)\n"
                                    << "                  z: FLOAT32 (z coordinate in meters)\n"
                                    << "                  timestamp: FLOAT64 (global time in seconds/nanoseconds if HESAI/LIVOX)\n"
                                    << "-------------------------------------------------------------------\n"
                                    );
        
            } else {
                RCLCPP_ERROR_STREAM(this->get_logger(), "\n-------------------------------------------------------------------\n" 
                                    << "lio_ros::FATAL ERROR: LiDAR sensor type unknown or not specified!\n"
                                    << "-------------------------------------------------------------------\n"
                                    );
            }
            
            return false;
        }

        visualization_msgs::msg::MarkerArray getMatchesMarker(std::vector<lio_ros::Match>& matches, std::string frame_id){
            visualization_msgs::msg::MarkerArray m_array;
            visualization_msgs::msg::Marker m;

            m_array.markers.reserve(matches.size()+1);

            m.header.frame_id = frame_id;
            m.header.stamp = this->get_clock()->now();
            m.ns = "lio_ros_match";
            
            m.id = -1;
            m.action = visualization_msgs::msg::Marker::DELETEALL;
            m_array.markers.push_back(m);

            m.type = visualization_msgs::msg::Marker::SPHERE;
            m.action = visualization_msgs::msg::Marker::ADD;

            m.color.r = 0.0f;
            m.color.g = 0.0f;
            m.color.b = 1.0f;
            m.color.a = 1.0f;

            m.lifetime = rclcpp::Duration::from_seconds(0.0);

            m.pose.orientation.w = 1.0;

            m.scale.x = 0.2;
            m.scale.y = 0.2;
            m.scale.z = 0.2;

            for(std::size_t i=0; i < matches.size(); i++){
                m.id = i;
                Eigen::Vector3f match_p = matches[i].point;
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

    auto lio_node = std::make_shared<LIOwrapper>();
    lio_node->init();

    rclcpp::executors::MultiThreadedExecutor executor; // by default using all available cores
    executor.add_node(lio_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}