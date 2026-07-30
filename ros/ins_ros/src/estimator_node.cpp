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

    // Set buffer capacity
    this->imu_buffer_.set_capacity(2000);
    this->state_buffer_.set_capacity(2000);

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

    state_buffer_.clear();
    imu_buffer_.clear();

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

    state_buffer_.clear();
    imu_buffer_.clear();

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
    if(imu_topic_.empty())
    {
        RCLCPP_ERROR(get_logger(), "IMU topic is not set. Please set 'topics.input.imu' parameter.");
        throw std::runtime_error("IMU topic not set");
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, 1000,
        std::bind(&INSEstimator::imu_callback, this, std::placeholders::_1));

    if(!gps_topic_.empty())
    {
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic_, 10,
            std::bind(&INSEstimator::gps_callback, this, std::placeholders::_1));
    }

    if(!wheel_odom_topic_.empty())
    {
        wheel_odom_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            wheel_odom_topic_, 10,
            std::bind(&INSEstimator::wheel_odom_callback, this, std::placeholders::_1));
    }

    if(!pose_topic_.empty())
    {
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 10,
            std::bind(&INSEstimator::pose_callback, this, std::placeholders::_1));
    }

    if(!mag_topic_.empty())
    {
        mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
            mag_topic_, 10,
            std::bind(&INSEstimator::mag_callback, this, std::placeholders::_1));
    }

    if(!baro_topic_.empty())
    {   
        baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
            baro_topic_, 10,
            std::bind(&INSEstimator::baro_callback, this, std::placeholders::_1));
    }
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

    this->filter_.setState(group); // set initial state
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////            CALLBACKS                /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void INSEstimator::imu_callback(const sensor_msgs::msg::Imu& msg)
{
    iESEKF::IMUmeas imu;
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
    filter_.predict(imu);

    // Update local state
    iESEKF::group_to_state(filter_.getState(), state_);
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
    using ins_ros::iESEKF::gps::WGS84_A;

    // Convert GPS message to latitude, longitude, altitude (lla)
    State::V3 llh;
    llh.x() = msg.latitude;
    llh.y() = msg.longitude;
    llh.z() = msg.altitude;

    // Convert LLA to ENU (relative to origin at state position)
    // Reference point: use state position as origin
    State::V3 llh_origin;
    // Convert state position (ENU) to LLA approximation
    // Note: This is a simplified conversion - use proper LLA conversion for accuracy
    llh_origin.x() = state_.p.y() / WGS84_A; // North (radians)
    llh_origin.y() = state_.p.x() / (WGS84_A * std::cos(llh_origin.x())); // East (radians)
    llh_origin.z() = -state_.p.z(); // Altitude (meters)

    State::V3 enu = ins_ros::iESEKF::gps::ecef_to_enu(
        ins_ros::iESEKF::gps::llh_to_ecef(llh),
        ins_ros::iESEKF::gps::llh_to_ecef(llh_origin)
    );

    // GPS measurement update (using meter covariance)
    iESEKF::Measurement meas(enu);

    // Measurement noise covariance (example values)
    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;
    Mat3 R_gps = Mat3::Identity() * 1.0; // 1m variance
    Mat3 R_gps_inv = R_gps.inverse();

    filter_.update<iESEKF::Measurement, iESEKF::Measurement, iESEKF::HMat>(
        meas,
        R_gps, R_gps_inv,
        ins_ros::iESEKF::gps::H_fun);

    // Update local state
    iESEKF::group_to_state(filter_.getState(), state_);
    state_.time = rclcpp::Time(msg.header.stamp).seconds();
}

void INSEstimator::wheel_odom_callback(const geometry_msgs::msg::TwistStamped& msg)
{
    iESEKF::Measurement meas = iESEKF::Measurement::Zero(3);
    meas(0) = msg.twist.linear.x;
    meas(1) = msg.twist.linear.y;
    // meas(2) = msg.twist.linear.z;

    // Wheel odometry measurement update
    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;
    Mat3 R_w     = Mat3::Identity() * gps_noise_ * gps_noise_; // example noise, adjust as needed
    Mat3 R_w_inv = R_w.inverse();

    filter_.update<iESEKF::Measurement, iESEKF::Measurement, iESEKF::HMat>(
        meas,
        R_w, R_w_inv,
        ins_ros::iESEKF::wheel::H_fun);

    // Update local state
    iESEKF::group_to_state(filter_.getState(), state_);
    state_.time = rclcpp::Time(msg.header.stamp).seconds();

    // Publish
    nav_msgs::msg::Odometry state_msg;
    from_ins_to_ros(state_, state_msg);
    state_pub_->publish(state_msg);
}

void INSEstimator::pose_callback(const geometry_msgs::msg::PoseStamped& msg)
{
    // Convert ROS pose message to manifold bundle (SGal3 + biases + gravity)
    State pose_meas;
    from_ros_to_ins(msg, pose_meas);

    iESEKF::Group group_meas;
    iESEKF::state_to_group(pose_meas, group_meas);

    // Measurement noise covariance (example values)
    // For pose measurement, we need to compute 6xDoF matrix
    Eigen::MatrixXd R_pose = Eigen::MatrixXd::Identity(6, 6) * 0.1; // 10cm/0.1rad covariance
    Eigen::MatrixXd R_pose_inv = R_pose.inverse();

    filter_.update<iESEKF::Group, iESEKF::Measurement, iESEKF::HMat>(
        group_meas,
        R_pose, R_pose_inv,
        ins_ros::iESEKF::pose::H_fun);

    // Update local state
    iESEKF::group_to_state(filter_.getState(), state_);
    state_.time = rclcpp::Time(msg.header.stamp).seconds();
}

void INSEstimator::mag_callback(const sensor_msgs::msg::MagneticField& msg)
{
    // Convert ROS magnetic field message to vector (Tesla)
    State::V3 mag_vector;
    mag_vector.x() = msg.magnetic_field.x;
    mag_vector.y() = msg.magnetic_field.y;
    mag_vector.z() = msg.magnetic_field.z;

    // Magnetic field measurement update
    iESEKF::Measurement meas(mag_vector);

    // Measurement noise covariance (example values)
    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;
    Mat3 R_mag = Mat3::Identity() * 0.05; // 0.05 Tesla variance
    Mat3 R_mag_inv = R_mag.inverse();

    filter_.update<iESEKF::Measurement, iESEKF::Measurement, iESEKF::HMat>(
        meas,
        R_mag, R_mag_inv,
        ins_ros::iESEKF::magnetometer::H_fun);

    // Update local state
    iESEKF::group_to_state(filter_.getState(), state_);
    state_.time = rclcpp::Time(msg.header.stamp).seconds();
}

void INSEstimator::baro_callback(const sensor_msgs::msg::FluidPressure& msg)
{
    // Get ROS pressure measurement
    iESEKF::Scalar pressure = msg.fluid_pressure; 

    // Measurement noise covariance (example values)
    Eigen::MatrixXd R_baro = Eigen::MatrixXd::Identity(1,1);
    R_baro(0, 0) = 2.0; // height measurement variance (2 meters equivalent)
    Eigen::MatrixXd R_baro_inv = R_baro.inverse();

    filter_.update<iESEKF::Scalar, iESEKF::Measurement, iESEKF::HMat>(
        pressure,
        R_baro, R_baro_inv,
        ins_ros::iESEKF::barometer::H_fun);

    // Update local state
    iESEKF::group_to_state(filter_.getState(), state_);
    state_.time = rclcpp::Time(msg.header.stamp).seconds();
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////        CONVERSION HELPERS           /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void INSEstimator::from_ros_to_ins(const sensor_msgs::msg::Imu& in, iESEKF::IMUmeas& out)
{
    out.stamp = rclcpp::Time(in.header.stamp).seconds();

    out.gyro(0) = in.angular_velocity.x;
    out.gyro(1) = in.angular_velocity.y;
    out.gyro(2) = in.angular_velocity.z;

    out.accel(0) = in.linear_acceleration.x;
    out.accel(1) = in.linear_acceleration.y;
    out.accel(2) = in.linear_acceleration.z;
}

void INSEstimator::from_ros_to_ins(const geometry_msgs::msg::PoseStamped& in, ins_ros::State& out)
{
    // Position
    out.p.x() = in.pose.position.x;
    out.p.y() = in.pose.position.y;
    out.p.z() = in.pose.position.z;

    // Orientation
    out.q.x() = static_cast<State::Scalar>(in.pose.orientation.x);
    out.q.y() = static_cast<State::Scalar>(in.pose.orientation.y);
    out.q.z() = static_cast<State::Scalar>(in.pose.orientation.z);
    out.q.w() = static_cast<State::Scalar>(in.pose.orientation.w);
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

void INSEstimator::from_ins_to_ros(const ins_ros::State& in, geometry_msgs::msg::PoseWithCovarianceStamped& out)
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

