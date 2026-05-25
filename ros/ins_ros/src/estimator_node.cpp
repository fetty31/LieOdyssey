#include "ins_ros/estimator_node.hpp"

namespace ins_ros {

INSEstimator::INSEstimator(const std::string& node_name)
    : LifecycleNode(node_name,
                    rclcpp::NodeOptions()
                        .automatically_declare_parameters_from_overrides(true))
    , filter_(iESEKF::MatDoF::Identity() * 1e-3,
              iESEKF::Filter::NoiseMatrix::Identity() * 1e-3,
              iESEKF::f,
              iESEKF::df_dx,
              iESEKF::df_dw,
              iESEKF::degeneracy_callback)
    , last_imu_stamp_(-1.0)
{
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////          LIFECYCLE TRANSITIONS        /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

INSEstimator::CallbackReturn INSEstimator::on_configure(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Configuring INSEstimator");

    load_parameters();
    setup_subscriptions();
    setup_publishers();

    // Reset filter
    filter_.reset();
    filter_.setCovariance(iESEKF::MatDoF::Identity() * 1e-3);
    filter_.setProcessNoise(iESEKF::Filter::NoiseMatrix::Identity() * 1e-3);
    filter_.setMaxIters(5);
    filter_.setTolerance(1e-9);

    // Initialize state 
    initState();

    // TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    last_imu_stamp_ = -1.0;

    RCLCPP_INFO(get_logger(), "INSEstimator configured");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_activate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Activating INSEstimator");

    state_pub_->on_activate();
    pose_pub_->on_activate();

    RCLCPP_INFO(get_logger(), "INSEstimator activated");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Deactivating INSEstimator");

    state_pub_->on_deactivate();
    pose_pub_->on_deactivate();

    RCLCPP_INFO(get_logger(), "INSEstimator deactivated");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_cleanup(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Cleaning up INSEstimator");

    imu_sub_.reset();
    gps_sub_.reset();
    wheel_odom_sub_.reset();
    pose_sub_.reset();
    mag_sub_.reset();
    baro_sub_.reset();
    state_pub_.reset();
    pose_pub_.reset();
    tf_broadcaster_.reset();

    filter_.reset();

    RCLCPP_INFO(get_logger(), "INSEstimator cleaned up");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_shutdown(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "Shutting down INSEstimator");

    imu_sub_.reset();
    gps_sub_.reset();
    wheel_odom_sub_.reset();
    pose_sub_.reset();
    mag_sub_.reset();
    baro_sub_.reset();
    state_pub_.reset();
    pose_pub_.reset();
    tf_broadcaster_.reset();

    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_error(const rclcpp_lifecycle::State&)
{
    RCLCPP_INFO(get_logger(), "INSEstimator error - cleaning up");

    imu_sub_.reset();
    gps_sub_.reset();
    wheel_odom_sub_.reset();
    pose_sub_.reset();
    mag_sub_.reset();
    baro_sub_.reset();
    state_pub_.reset();
    pose_pub_.reset();
    tf_broadcaster_.reset();

    return CallbackReturn::SUCCESS;
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////           SETUP HELPERS              /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void INSEstimator::load_parameters()
{
    // Frames
    world_frame_ = get_parameter("frames.world").as_string();
    body_frame_  = get_parameter("frames.body").as_string();
    publish_tf_  = get_parameter("frames.tf_pub").as_bool();

    // Topics
    imu_topic_ = get_parameter("topics.input.imu").as_string();
    gps_topic_ = get_parameter("topics.input.gps").as_string();
    wheel_odom_topic_ = get_parameter("topics.input.wheel_odom").as_string();
    pose_topic_ = get_parameter("topics.input.pose").as_string();
    mag_topic_ = get_parameter("topics.input.mag").as_string();
    baro_topic_ = get_parameter("topics.input.baro").as_string();

    // Process noise (IMU)
    gyro_noise_      = get_parameter("iESEKF.covariance.gyro").as_double();
    accel_noise_     = get_parameter("iESEKF.covariance.accel").as_double();
    gyro_bias_noise_ = get_parameter("iESEKF.covariance.bias_gyro").as_double();
    accel_bias_noise_= get_parameter("iESEKF.covariance.bias_accel").as_double();

    // Measurement noise
    // gps_noise_ = get_parameter("iESEKF.covariance.gps").as_double();
    gps_noise_ = 0.01; // example value, adjust as needed
}

void INSEstimator::setup_subscriptions()
{
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 1000,
        std::bind(&INSEstimator::imu_callback, this, std::placeholders::_1));

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
        gps_topic_, 10,
        std::bind(&INSEstimator::gps_callback, this, std::placeholders::_1));

    wheel_odom_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        wheel_odom_topic_, 10,
        std::bind(&INSEstimator::wheel_odom_callback, this, std::placeholders::_1));

    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_topic_, 10,
        std::bind(&INSEstimator::pose_callback, this, std::placeholders::_1));

    mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
        mag_topic_, 10,
        std::bind(&INSEstimator::mag_callback, this, std::placeholders::_1));

    baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
        baro_topic_, 10,
        std::bind(&INSEstimator::baro_callback, this, std::placeholders::_1));
}

void INSEstimator::setup_publishers()
{
    state_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ins_ros/odom", 1);
    pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ins_ros/pose", 1);
}

void INSEstimator::initState() 
{
    iESEKF::Group group;
    iESEKF::state_to_group(this->state_, group);

    group_ = group; // store initial group state (debugging)

    this->filter_.setState(group); // set initial state
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////            CALLBACKS                /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void INSEstimator::imu_callback(const sensor_msgs::msg::Imu& msg)
{
    lie_odyssey::IMUmeas imu;
    from_ros_to_ins(msg, imu);

    // Compute dt from last IMU stamp
    if (last_imu_stamp_ < 0.0)
    {
        last_imu_stamp_ = imu.stamp;
        return;
    }
    imu.dt = imu.stamp - last_imu_stamp_;
    last_imu_stamp_ = imu.stamp;

    // Filter prediction
    // filter_.predict(imu);
    group_.plus(iESEKF::f(filter_, imu) * imu.dt ); // manual integration for debugging

    // Update local state
    // iESEKF::group_to_state(filter_.getState(), state_);
    iESEKF::group_to_state(group_, state_);
    state_.time = imu.stamp;

    // Publish
    nav_msgs::msg::Odometry state_msg;
    from_ins_to_ros(state_, state_msg);
    state_pub_->publish(state_msg);

    // TF
    if (publish_tf_)
        broadcast_tf(state_);
}

void INSEstimator::gps_callback(const sensor_msgs::msg::NavSatFix& msg)
{
    (void)msg;

    // GPS measurement update
    Eigen::MatrixXd R_gps    = Eigen::Matrix3d::Identity() * gps_noise_;
    Eigen::MatrixXd R_gps_inv= R_gps.inverse();

    filter_.update<iESEKF::Measurement, iESEKF::HMat>(
        R_gps, R_gps_inv,
        ins_ros::iESEKF::gps::H_fun);

    // Update local state
    iESEKF::group_to_state(filter_.getState(), state_);
}

void INSEstimator::wheel_odom_callback(const geometry_msgs::msg::TwistStamped& msg)
{
    (void)msg;
    // To-Do: implement wheel odometry update
}

void INSEstimator::pose_callback(const geometry_msgs::msg::PoseStamped& msg)
{
    (void)msg;
    // To-Do: implement pose measurement update (e.g., from VIO, LIO, etc.)
}

void INSEstimator::mag_callback(const sensor_msgs::msg::MagneticField& msg)
{
    (void)msg;
    // To-Do: implement magnetometer measurement update
}

void INSEstimator::baro_callback(const sensor_msgs::msg::FluidPressure& msg)
{
    (void)msg;
    // To-Do: implement barometer measurement update
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////        CONVERSION HELPERS           /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void INSEstimator::from_ros_to_ins(const sensor_msgs::msg::Imu& in, lie_odyssey::IMUmeas& out)
{
    out.stamp = rclcpp::Time(in.header.stamp).seconds();

    out.gyro(0) = in.angular_velocity.x;
    out.gyro(1) = in.angular_velocity.y;
    out.gyro(2) = in.angular_velocity.z;

    out.accel(0) = in.linear_acceleration.x;
    out.accel(1) = in.linear_acceleration.y;
    out.accel(2) = in.linear_acceleration.z;
}

void INSEstimator::from_ins_to_ros(const ins_ros::State& in, nav_msgs::msg::Odometry& out)
{
    out.header.stamp = rclcpp::Time(in.time);
    out.header.frame_id = world_frame_;
    out.child_frame_id  = body_frame_;

    out.pose.pose.position.x = in.p(0);
    out.pose.pose.position.y = in.p(1);
    out.pose.pose.position.z = in.p(2);

    out.pose.pose.orientation.x = in.q.x();
    out.pose.pose.orientation.y = in.q.y();
    out.pose.pose.orientation.z = in.q.z();
    out.pose.pose.orientation.w = in.q.w();

    out.twist.twist.linear.x  = in.v(0);
    out.twist.twist.linear.y  = in.v(1);
    out.twist.twist.linear.z  = in.v(2);

    out.twist.twist.angular.x = in.w(0);
    out.twist.twist.angular.y = in.w(1);
    out.twist.twist.angular.z = in.w(2);

    // Covariances (row-major 6x6)
    auto pose_cov  = iESEKF::get_pose_covariance(filter_.getCovariance());
    auto twist_cov = iESEKF::get_velocity_covariance(filter_.getCovariance());
    for (int i = 0; i < 36; ++i)
    {
        out.pose.covariance[i]  = pose_cov[i];
        out.twist.covariance[i] = twist_cov[i];  
    }
    // add gyro covariance
    out.twist.covariance[21] = gyro_noise_;
    out.twist.covariance[28] = gyro_noise_;
    out.twist.covariance[25] = gyro_noise_;
}

void INSEstimator::from_ins_to_ros_pose(const ins_ros::State& in, geometry_msgs::msg::PoseWithCovarianceStamped& out)
{
    // Position
    out.pose.pose.position.x = in.p(0);
    out.pose.pose.position.y = in.p(1);
    out.pose.pose.position.z = in.p(2);

    // Orientation
    out.pose.pose.orientation.x = in.q.x();
    out.pose.pose.orientation.y = in.q.y();
    out.pose.pose.orientation.z = in.q.z();
    out.pose.pose.orientation.w = in.q.w();

    // Covariance (row-major 6x6)
    auto pose_cov  = iESEKF::get_pose_covariance(filter_.getCovariance());
    for (int i = 0; i < 36; ++i)
    {
        out.pose.covariance[i]  = pose_cov[i];
    }
}

void INSEstimator::broadcast_tf(const ins_ros::State& in, bool now)
{
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = (now) ? this->get_clock()->now() : rclcpp::Time(in.time);
    tf_msg.header.frame_id = world_frame_;
    tf_msg.child_frame_id  = body_frame_;

    tf_msg.transform.translation.x = in.p(0);
    tf_msg.transform.translation.y = in.p(1);
    tf_msg.transform.translation.z = in.p(2);

    tf_msg.transform.rotation.x = in.q.x();
    tf_msg.transform.rotation.y = in.q.y();
    tf_msg.transform.rotation.z = in.q.z();
    tf_msg.transform.rotation.w = in.q.w();

    tf_broadcaster_->sendTransform(tf_msg);
}

} // namespace ins_ros

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ins_ros::INSEstimator>();

    // rclcpp::executors::MultiThreadedExecutor executor;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}

