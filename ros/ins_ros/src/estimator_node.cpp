#include "ins_ros/estimator_node.hpp"

namespace ins_ros {

INSEstimator::INSEstimator(const std::string& node_name)
    : LifecycleNode(node_name)
    , filter_(iESEKF::MatDoF::Identity() * 1e-3,
              iESEKF::Filter::NoiseMatrix::Identity() * 1e-3,
              iESEKF::f_cv,
              iESEKF::df_dx_cv,
              iESEKF::df_dw_cv,
              iESEKF::degeneracy_callback)
    , tf_buffer_(this->get_clock())
    , imu_to_base_(tf_buffer_, get_logger())
    , lio_to_base_(tf_buffer_, get_logger())
    , initial_enu_base_(tf_buffer_, get_logger())
    , lio_to_enu_(tf_buffer_, get_logger())
    , last_imu_stamp_(-1.0)
{
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////          LIFECYCLE TRANSITIONS        /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

INSEstimator::CallbackReturn INSEstimator::on_configure(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Configuring...");

    // Initialize state 
    this->state_ = ins_ros::State();

    // Set buffer capacity
    this->imu_buffer_.set_capacity(2000);
    this->state_buffer_.set_capacity(2000);

    // Reset ENU frame
    enu_converter_ = ENUConverter();

    // TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // Frame transforms
    imu_to_base_.reset();
    lio_to_base_.reset();
    initial_enu_base_.reset();
    lio_to_enu_.reset();

    // IMU tracking
    previous_omega_base_ = State::V3::Zero();
    last_imu_stamp_ = -1.0;

    // Orientation initializers
    imu_orientation_initializer_ = std::make_unique<init::IMUOrientationInitializer>();
    gps_orientation_initializer_ = std::make_unique<init::GPSOrientationInitializer>();
    orientation_initialized_ = false;

        // debug
    init::GPSOrientationInitializer::Parameters params;
    params.max_speed = 20.5;
    gps_orientation_continuous_ = std::make_unique<init::GPSOrientationInitializer>(params);
    
    // Load parameters, setup subscriptions and publishers
    declare_parameters();
    load_parameters();
    setup_subscriptions();
    setup_publishers();

    // Reset filter
    filter_.reset();
    filter_.setCovariance(iESEKF::MatDoF::Identity() * 1e-3);

    iESEKF::Filter::NoiseMatrix Q = iESEKF::Filter::NoiseMatrix::Identity();
    Q.block<3, 3>(0, 0) = static_cast<iESEKF::Scalar>(gyro_noise_) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
    Q.block<3, 3>(3, 3) = static_cast<iESEKF::Scalar>(accel_noise_) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
    Q.block<3, 3>(6, 6) = static_cast<iESEKF::Scalar>(gyro_bias_noise_) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
    Q.block<3, 3>(9, 9) = static_cast<iESEKF::Scalar>(accel_bias_noise_) * Eigen::Matrix<iESEKF::Scalar, 3, 3>::Identity();
    filter_.setProcessNoise(Q);

    filter_.setMaxIters(max_iters_);
    filter_.setTolerance(tolerance_);

    setState();

    // Set time reference
    t0_system_ = std::chrono::steady_clock::now();

    RCLCPP_INFO(get_logger(), "Configured");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_activate(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Activating...");

    state_pub_->on_activate();
    pose_pub_->on_activate();

    RCLCPP_INFO(get_logger(), "Activated");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating...");

    state_pub_->on_deactivate();
    pose_pub_->on_deactivate();

    RCLCPP_INFO(get_logger(), "Deactivated");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_cleanup(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Cleaning up...");

    imu_sub_.reset();
    gps_sub_.reset();
    wheel_odom_sub_.reset();
    odom_sub_.reset();
    mag_sub_.reset();
    baro_sub_.reset();
    state_pub_.reset();
    pose_pub_.reset();
    tf_broadcaster_.reset();

    filter_.reset();

    state_buffer_.clear();
    imu_buffer_.clear();

    RCLCPP_INFO(get_logger(), "Cleaned up");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_shutdown(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Shutting down...");

    imu_sub_.reset();
    gps_sub_.reset();
    wheel_odom_sub_.reset();
    odom_sub_.reset();
    mag_sub_.reset();
    baro_sub_.reset();
    state_pub_.reset();
    pose_pub_.reset();
    tf_broadcaster_.reset();

    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_error(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Error occurred - cleaning up...");

    imu_sub_.reset();
    gps_sub_.reset();
    wheel_odom_sub_.reset();
    odom_sub_.reset();
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

void INSEstimator::declare_parameters()
{
    // Frames
    declare_parameter<std::string>("frames.world", "odom");
    declare_parameter<std::string>("frames.body", "base_link");

    declare_parameter<bool>("tf.publish", true);

    // Filter
    declare_parameter<int>("filter.iterations.max", 5);
    declare_parameter<double>("filter.iterations.tolerance", 1e-6);

    declare_parameter<double>("filter.process_noise.gyro", 6.01e-4);
    declare_parameter<double>("filter.process_noise.accel", 1.53e-2);

    declare_parameter<double>("filter.process_noise.gyro_bias", 1.54e-5);
    declare_parameter<double>("filter.process_noise.accel_bias", 3.38e-4);

    // IMU
    declare_parameter<bool>("sensors.imu.enabled", true);
    declare_parameter<std::string>("sensors.imu.topic", "/imu/data");
    declare_parameter<bool>("sensors.imu.estimate_bias", false);
    declare_parameter<bool>("sensors.imu.estimate_orientation", false);

        // Fixed IMU biases
    declare_parameter<double>("sensors.imu.bias.accel.x", 0.0);
    declare_parameter<double>("sensors.imu.bias.accel.y", 0.0);
    declare_parameter<double>("sensors.imu.bias.accel.z", 0.0);

    declare_parameter<double>("sensors.imu.bias.gyro.x", 0.0);
    declare_parameter<double>("sensors.imu.bias.gyro.y", 0.0);
    declare_parameter<double>("sensors.imu.bias.gyro.z", 0.0);

    // GPS
    declare_parameter<bool>("sensors.gps.enabled", false);
    declare_parameter<std::string>("sensors.gps.topic", "/gps/fix");
    declare_parameter<bool>("sensors.gps.trust_covariance", true);

        // GPS position noise
    declare_parameter<double>("sensors.gps.covariance.position.x", 1.0);
    declare_parameter<double>("sensors.gps.covariance.position.y", 1.0);
    declare_parameter<double>("sensors.gps.covariance.position.z", 2.0);

        // GPS lever arm: IMU/body -> GPS antenna, expressed in body frame
    declare_parameter<double>("sensors.gps.lever_arm.x", 0.0);
    declare_parameter<double>("sensors.gps.lever_arm.y", 0.0);
    declare_parameter<double>("sensors.gps.lever_arm.z", 0.0);

    // 3D Odometry
    declare_parameter<bool>("sensors.odometry.enabled", false);
    declare_parameter<std::string>("sensors.odometry.topic", "/odometry");
    declare_parameter<bool>("sensors.odometry.trust_covariance.pose", true);
    declare_parameter<bool>("sensors.odometry.trust_covariance.velocity", true);

    declare_parameter<double>("sensors.odometry.covariance.position.x", 0.1);
    declare_parameter<double>("sensors.odometry.covariance.position.y", 0.1);
    declare_parameter<double>("sensors.odometry.covariance.position.z", 0.1);

    declare_parameter<double>("sensors.odometry.covariance.orientation.x", 0.01);
    declare_parameter<double>("sensors.odometry.covariance.orientation.y", 0.01);
    declare_parameter<double>("sensors.odometry.covariance.orientation.z", 0.01);

    declare_parameter<double>("sensors.odometry.covariance.velocity.x", 0.1);
    declare_parameter<double>("sensors.odometry.covariance.velocity.y", 0.1);
    declare_parameter<double>("sensors.odometry.covariance.velocity.z", 0.1);

    // Wheel odometry
    declare_parameter<bool>("sensors.wheel_odom.enabled", false);
    declare_parameter<std::string>("sensors.wheel_odom.topic", "/wheel/odometry");

        // velocity noise
    declare_parameter<double>("sensors.wheel_odom.covariance.velocity.x", 0.1);
    declare_parameter<double>("sensors.wheel_odom.covariance.velocity.y", 0.1);
    declare_parameter<double>("sensors.wheel_odom.covariance.velocity.z", 0.1);

    // Barometer
    declare_parameter<bool>("sensors.baro.enabled", false);
    declare_parameter<std::string>("sensors.baro.topic", "/baro");
    declare_parameter<double>("sensors.baro.covariance.altitude", 1.0);

    // Magnetometer
    declare_parameter<bool>("sensors.mag.enabled", false);
    declare_parameter<std::string>("sensors.mag.topic", "/mag");
    declare_parameter<double>("sensors.mag.covariance.heading", 0.1);

}

void INSEstimator::load_parameters()
{
    // Frames
    world_frame_ = get_parameter("frames.world").as_string();
    body_frame_  = get_parameter("frames.body").as_string();
    publish_tf_  = get_parameter("tf.publish").as_bool();

    // Filter
    max_iters_ = get_parameter("filter.iterations.max").as_int();
    tolerance_ = get_parameter("filter.iterations.tolerance").as_double();

    // Sensors
        // IMU
    bool imu_enabled = get_parameter("sensors.imu.enabled").as_bool();
    if (!imu_enabled)
    {
        RCLCPP_ERROR(get_logger(), "IMU is disabled. This estimator requires an IMU. Please enable the IMU in the parameters.");
        throw std::runtime_error("IMU is disabled");
    }
    imu_topic_ = get_parameter("sensors.imu.topic").as_string();
    estimate_imu_bias_ = get_parameter("sensors.imu.estimate_bias").as_bool();
    estimate_imu_orientation_ = get_parameter("sensors.imu.estimate_orientation").as_bool();

    if(!estimate_imu_bias_)
    {
        RCLCPP_WARN(get_logger(), "IMU bias estimation is disabled. The estimator will run with fixed IMU biases.");
        state_.bias.a(0) = get_parameter("sensors.imu.bias.accel.x").as_double();
        state_.bias.a(1) = get_parameter("sensors.imu.bias.accel.y").as_double();
        state_.bias.a(2) = get_parameter("sensors.imu.bias.accel.z").as_double();
        state_.bias.w(0) = get_parameter("sensors.imu.bias.gyro.x").as_double();
        state_.bias.w(1) = get_parameter("sensors.imu.bias.gyro.y").as_double();
        state_.bias.w(2) = get_parameter("sensors.imu.bias.gyro.z").as_double();
        if(!estimate_imu_orientation_){
            // If both bias and orientation estimation are disabled, we can assume the IMU is perfectly calibrated.
            imu_orientation_initializer_->initialized_ = true;
            RCLCPP_WARN(get_logger(), "IMU orientation initialization is disabled. Assuming roll=0º and pitch=0º.");
        }
    }

        // GPS
    gps_topic_.clear();

    bool gps_enabled = get_parameter("sensors.gps.enabled").as_bool();
    if (!gps_enabled)
    {
        RCLCPP_WARN(get_logger(), "GPS is disabled. The estimator will run without GPS.");
    }else{
        gps_topic_ = get_parameter("sensors.gps.topic").as_string();
        trust_gps_covariance_ = get_parameter("sensors.gps.trust_covariance").as_bool();
        double gps_noise_x = get_parameter("sensors.gps.covariance.position.x").as_double();
        double gps_noise_y = get_parameter("sensors.gps.covariance.position.y").as_double();
        double gps_noise_z = get_parameter("sensors.gps.covariance.position.z").as_double();
        gps_noise_ = State::V3(gps_noise_x, gps_noise_y, gps_noise_z);
        double lever_arm_x = get_parameter("sensors.gps.lever_arm.x").as_double();
        double lever_arm_y = get_parameter("sensors.gps.lever_arm.y").as_double();
        double lever_arm_z = get_parameter("sensors.gps.lever_arm.z").as_double();
        gps_lever_arm_ = State::V3(lever_arm_x, lever_arm_y, lever_arm_z);
    }
    
        // 3D Odometry
    odom_topic_.clear();
    bool odom_enabled = get_parameter("sensors.odometry.enabled").as_bool();
    if (odom_enabled)
    {
        odom_topic_ = get_parameter("sensors.odometry.topic").as_string();
        trust_odom_pose_covariance_ = get_parameter("sensors.odometry.trust_covariance.pose").as_bool();
        trust_odom_velocity_covariance_ = get_parameter("sensors.odometry.trust_covariance.velocity").as_bool();
        double odom_position_noise_x = get_parameter("sensors.odometry.covariance.position.x").as_double();
        double odom_position_noise_y = get_parameter("sensors.odometry.covariance.position.y").as_double();
        double odom_position_noise_z = get_parameter("sensors.odometry.covariance.position.z").as_double();
        odom_position_noise_ = State::V3(odom_position_noise_x, odom_position_noise_y, odom_position_noise_z);
        double odom_orientation_noise_x = get_parameter("sensors.odometry.covariance.orientation.x").as_double();
        double odom_orientation_noise_y = get_parameter("sensors.odometry.covariance.orientation.y").as_double();
        double odom_orientation_noise_z = get_parameter("sensors.odometry.covariance.orientation.z").as_double();
        odom_orientation_noise_ = State::V3(odom_orientation_noise_x, odom_orientation_noise_y, odom_orientation_noise_z);
        double odom_velocity_noise_x = get_parameter("sensors.odometry.covariance.velocity.x").as_double();
        double odom_velocity_noise_y = get_parameter("sensors.odometry.covariance.velocity.y").as_double();
        double odom_velocity_noise_z = get_parameter("sensors.odometry.covariance.velocity.z").as_double();
        odom_velocity_noise_ = State::V3(odom_velocity_noise_x, odom_velocity_noise_y, odom_velocity_noise_z);
    }

        // Wheel odometry
    wheel_odom_topic_.clear();
    bool wheel_odom_enabled = get_parameter("sensors.wheel_odom.enabled").as_bool();
    if (wheel_odom_enabled)
    {
        wheel_odom_topic_ = get_parameter("sensors.wheel_odom.topic").as_string();
        double wheel_odom_noise_x = get_parameter("sensors.wheel_odom.covariance.velocity.x").as_double();
        double wheel_odom_noise_y = get_parameter("sensors.wheel_odom.covariance.velocity.y").as_double();
        double wheel_odom_noise_z = get_parameter("sensors.wheel_odom.covariance.velocity.z").as_double();
        wheel_odom_noise_ = State::V3(wheel_odom_noise_x, wheel_odom_noise_y, wheel_odom_noise_z);
    }
    
        // Magnetometer
    mag_topic_.clear();
    bool mag_enabled = get_parameter("sensors.mag.enabled").as_bool();
    if (mag_enabled)
    {
        mag_topic_ = get_parameter("sensors.mag.topic").as_string();
    }

        // Barometer
    baro_topic_.clear();
    bool baro_enabled = get_parameter("sensors.baro.enabled").as_bool();
    if (baro_enabled)
    {
        baro_topic_ = get_parameter("sensors.baro.topic").as_string();
    }

    // Process noise (IMU)
    gyro_noise_      = get_parameter("filter.process_noise.gyro").as_double();
    accel_noise_     = get_parameter("filter.process_noise.accel").as_double();
    gyro_bias_noise_ = get_parameter("filter.process_noise.gyro_bias").as_double();
    accel_bias_noise_= get_parameter("filter.process_noise.accel_bias").as_double();

    RCLCPP_INFO(get_logger(), "Parameters loaded.");
}

void INSEstimator::setup_subscriptions()
{
    RCLCPP_INFO(get_logger(), "Subscribed to:");

    const auto imu_qos = rclcpp::QoS(rclcpp::KeepLast(1000))
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    const auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10))
        .reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT)
        .durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    if(imu_topic_.empty())
    {
        RCLCPP_ERROR(get_logger(), "IMU topic is not set. Please set 'topics.input.imu' parameter.");
        throw std::runtime_error("IMU topic not set");
    }
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_,
        imu_qos,
        std::bind(&INSEstimator::imu_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "  IMU:        %s", imu_topic_.c_str());

    if(!gps_topic_.empty())
    {
        gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
            gps_topic_,
            sensor_qos,
            std::bind(&INSEstimator::gps_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "  GPS:        %s", gps_topic_.c_str());
    }

    if(!wheel_odom_topic_.empty())
    {
        wheel_odom_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
            wheel_odom_topic_, 
            sensor_qos,
            std::bind(&INSEstimator::wheel_odom_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "  Wheel odom: %s", wheel_odom_topic_.c_str());
    }

    if(!odom_topic_.empty())
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, 
            sensor_qos,
            std::bind(&INSEstimator::odom_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "  3D Odometry: %s", odom_topic_.c_str());
    }

    if(!mag_topic_.empty())
    {
        mag_sub_ = create_subscription<sensor_msgs::msg::MagneticField>(
            mag_topic_, 
            sensor_qos,
            std::bind(&INSEstimator::mag_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "  Magnetometer: %s", mag_topic_.c_str());
    }

    if(!baro_topic_.empty())
    {   
        baro_sub_ = create_subscription<sensor_msgs::msg::FluidPressure>(
            baro_topic_, 
            sensor_qos,
            std::bind(&INSEstimator::baro_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "  Barometer:  %s", baro_topic_.c_str());
    }
}

void INSEstimator::setup_publishers()
{
    state_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/odom", 1);
    pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("~/pose", 1);

    debug_gps_pub_ =
        create_publisher<visualization_msgs::msg::Marker>(
            "~/debug/gps_position", 10);
    debug_odom_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(
            "~/debug/lio_odom", 10);
    debug_yaw_pub_ = create_publisher<visualization_msgs::msg::Marker>(
        "~/debug/yaw", 10);
}

void INSEstimator::setState() 
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

    // Compute time difference
    if (last_imu_stamp_ < 0.0)
    {
        last_imu_stamp_ = imu.stamp;
        imu.dt = 0.0;
    }
    else
    {
        imu.dt = imu.stamp - last_imu_stamp_;
        last_imu_stamp_ = imu.stamp;
    }

    // Transform IMU measurements into base/body frame if necessary.
    if (!transform_imu_to_base_link(msg, imu))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Skipping IMU propagation.");
        return;
    }

    if (imu.dt <= 0.0 || imu.dt >= 0.1)
    {
        previous_omega_base_ = imu.gyro;
        return;
    }

    // Initialize IMU orientation if not already done
    if (!imu_orientation_initializer_->initialized())
    {
        Eigen::Vector3d accel = imu.accel.cast<double>();
        Eigen::Vector3d gyro = imu.gyro.cast<double>();

        RCLCPP_INFO_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Estimating IMU orientation from accelerometer data...");

        if (imu_orientation_initializer_->add_measurement(accel, gyro))
        {
            if(estimate_imu_bias_){
                state_.bias.a = imu_orientation_initializer_->accelerometer_bias().cast<iESEKF::Scalar>();
                state_.bias.w = imu_orientation_initializer_->gyroscope_bias().cast<iESEKF::Scalar>();
            }
            RCLCPP_INFO(
                get_logger(),
                "IMU orientation computed (roll/pitch) with accelerometer bias: [%.3f, %.3f, %.3f] m/s^2 and gyroscope bias: [%.3f, %.3f, %.3f] rad/s",
                state_.bias.a.x(),
                state_.bias.a.y(),
                state_.bias.a.z(),
                state_.bias.w.x(),
                state_.bias.w.y(),
                state_.bias.w.z());
        }else{
            RCLCPP_DEBUG(
                get_logger(),
                "IMU orientation initializer: %zu/%zu samples collected",
                imu_orientation_initializer_->sample_count_,
                imu_orientation_initializer_->params_.min_samples);
            return;
        }
    }

    // If the GPS is active, the filter state will be expressed in a local ENU frame
    // for now, we need to ensure that the ENU origin has been initialized before propagating (to-do: handle this better)
    if (!gps_topic_.empty())
    {
        if (!enu_converter_.initialized())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Local ENU frame not initialized. Skipping IMU propagation.");
            return;
        }
    }

    // Check whether complete orientation is available
    if (!orientation_initialized_)
    {
        if (!imu_orientation_initializer_->initialized())
            return;

        if ( (!gps_orientation_initializer_->initialized()) && (!gps_topic_.empty()) )
            return;

        initialize_orientation();
    }

    RCLCPP_DEBUG_THROTTLE(
        get_logger(),
        *get_clock(),
        1000,
        "Propagating state with corrected IMU measurement: "
        "accel=[%.3f, %.3f, %.3f] m/s², "
        "gyro=[%.3f, %.3f, %.3f] rad/s, "
        "dt=%.6f s",
        imu.accel.x() - state_.bias.a.x(),
        imu.accel.y() - state_.bias.a.y(),
        imu.accel.z() - state_.bias.a.z(),
        imu.gyro.x() - state_.bias.w.x(),
        imu.gyro.y() - state_.bias.w.y(),
        imu.gyro.z() - state_.bias.w.z(),
        imu.dt);

    // Filter prediction
    // print_state("Before IMU propagation", state_);
    filter_.predict(imu);

    // Update local state
    const auto now_system = std::chrono::steady_clock::now();
    const double now =
        std::chrono::duration<double>(now_system - t0_system_).count();
    iESEKF::group_to_state(filter_.getState(), now, state_);
    state_.w = imu.gyro;
    state_.a = imu.accel;

    // print_state("After IMU propagation", state_);

    // Publish
    publish_odom();
    publish_pose();

    // TF
    if (publish_tf_)
        broadcast_tf(state_);
}

void INSEstimator::gps_callback(
    const sensor_msgs::msg::NavSatFix& msg)
{
    if (msg.status.status <
        sensor_msgs::msg::NavSatStatus::STATUS_FIX)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Ignoring GPS measurement without valid fix");

        return;
    }

    // First valid GPS fixes the ENU datum.
    if (!enu_converter_.initialized())
    {
        enu_converter_.set_origin(
            msg.latitude,
            msg.longitude,
            msg.altitude);

        RCLCPP_INFO(
            get_logger(),
            "ENU origin initialized at "
            "lat=%.8f lon=%.8f alt=%.3f",
            msg.latitude,
            msg.longitude,
            msg.altitude);

        return;
    }

    // GPS LLA -> local ENU
    const Eigen::Vector3d p_gps_enu =
        enu_converter_.to_enu(
            msg.latitude,
            msg.longitude,
            msg.altitude);

    publish_gps_debug(p_gps_enu);

    // Initialize GPS orientation if not already done
    if (!gps_orientation_initializer_->initialized())
    {
        if (gps_orientation_initializer_->add_position(p_gps_enu, 
                                                    rclcpp::Time(msg.header.stamp).seconds()))
        {
            RCLCPP_INFO(
                get_logger(),
                "GPS orientation initialized (yaw)");
        }else{
            RCLCPP_DEBUG(
                get_logger(),
                "GPS orientation initializer: %f m traveled",
                gps_orientation_initializer_->distance_traveled_);
            return;
        }
    }

    if(!orientation_initialized_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "Orientation not initialized. Skipping GPS measurement.");
        return;
    }

    iESEKF::gps::GPSMeasurement meas;
    meas.position_enu = p_gps_enu.cast<iESEKF::Scalar>();
    meas.lever_arm = gps_lever_arm_.cast<iESEKF::Scalar>();

    // RCLCPP_DEBUG(
    //     get_logger(),
    //     "GPS measurement in ENU frame: [%.3f, %.3f, %.3f] m",
    //     meas.position_enu.x(),
    //     meas.position_enu.y(),
    //     meas.position_enu.z());

    using Mat3 =
        Eigen::Matrix<iESEKF::Scalar, 3, 3>;

    Mat3 R_gps;

    if ((msg.position_covariance_type !=
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) && trust_gps_covariance_)
    {
        // Assuming covariance expressed in ENU frame (ROS convention)
        R_gps <<
            msg.position_covariance[0],
            msg.position_covariance[1],
            msg.position_covariance[2],

            msg.position_covariance[3],
            msg.position_covariance[4],
            msg.position_covariance[5],

            msg.position_covariance[6],
            msg.position_covariance[7],
            msg.position_covariance[8];
    }
    else
    {
        R_gps = Mat3::Zero();
        R_gps(0, 0) = gps_noise_.x();
        R_gps(1, 1) = gps_noise_.y();
        R_gps(2, 2) = gps_noise_.z();
    }

    const Mat3 R_gps_inv =
        R_gps.inverse();

    filter_.update<
        iESEKF::gps::GPSMeasurement,
        iESEKF::Measurement,
        iESEKF::HMat>(
            meas,
            R_gps,
            R_gps_inv,
            ins_ros::iESEKF::gps::H_fun);

    // Update local state
    const auto now_system = std::chrono::steady_clock::now();
    const double now =
        std::chrono::duration<double>(now_system - t0_system_).count();
    iESEKF::group_to_state(filter_.getState(), now, state_);
    
    print_state("After GPS update", state_);

    if(gps_orientation_continuous_->add_position(p_gps_enu, rclcpp::Time(msg.header.stamp).seconds()))
    {
        double yaw = gps_orientation_continuous_->heading();
        Eigen::Matrix<iESEKF::Scalar, 2, 2> R_yaw, R_yaw_inv;
        R_yaw.setIdentity();
        R_yaw *= 0.01;
        R_yaw_inv = R_yaw.inverse();
        filter_.update<
        iESEKF::Scalar,
        iESEKF::Measurement,
        iESEKF::HMat>(
            yaw,
            R_yaw,
            R_yaw_inv,
            ins_ros::iESEKF::yaw::H_fun);

        RCLCPP_DEBUG(get_logger(), "Updating yaw: %f", yaw*180.0/M_PI);
        
        visualization_msgs::msg::Marker marker;

        marker.header.frame_id = world_frame_;
        marker.header.stamp = this->now();
        marker.ns = "yaw_marker";
        marker.id =  0;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = p_gps_enu.x();
        marker.pose.position.y = p_gps_enu.y();
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        marker.pose.orientation = tf2::toMsg(q);
        marker.scale.x = 1.0;
        marker.scale.y = 0.06;
        marker.scale.z = 0.06;
        marker.color.r = 1.0f;
        marker.color.g = 0.0f;
        marker.color.b = 1.0f;
        marker.color.a = 1.0;
        marker.lifetime = rclcpp::Duration::from_nanoseconds(0);
        debug_yaw_pub_->publish(marker);

        gps_orientation_continuous_->reset();
    }
}

void INSEstimator::odom_callback(const nav_msgs::msg::Odometry& msg)
{
    // Initialize LIO/VIO-body -> EKF-body extrinsics
    if (!lio_to_base_.initialized())
    {
        if (!lio_to_base_.initialize(msg.child_frame_id, body_frame_))
        {
            RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Skipping ODOMETRY (LIO/VIO) measurement.");
            return;
        }
    }

    // Convert ROS Odometry -> INS State
    //
    // i.e. the pose of the LIO body expressed in the
    // arbitrary LIO world frame.
    State odom_meas;
    from_ros_to_ins(msg, odom_meas);

    odom_meas.v = odom_meas.q.toRotationMatrix() * odom_meas.v; // convert velocity to inertial frame

    const auto now_system = std::chrono::steady_clock::now();
    const double now =
        std::chrono::duration<double>(now_system - t0_system_).count();
    odom_meas.time = now;

    // Transform LIO/VIO world -> ENU
    //
    // The incoming LIO/VIO pose is expressed in an
    // arbitrary LIO/VIO world frame. If GPS is active, we need to align that world frame
    // with the INS ENU frame.
    //
    // Assuming:
    //   - LIO/VIO and INS use the same IMU,
    //   - LIO/VIO initializes roll/pitch from the IMU,
    //   - LIO/VIO starts with yaw = 0,
    //   - the transform from the LIO/VIO body to the INS base frame is known
    // we can determine the initial LIO/VIO-world <-> ENU rotation
    // from the initial ENU/base orientation.
    //
    // In particular, initial_enu_base_ gives the initial transform
    // of the filter body frame in ENU. Combining it with the initial
    // LIO/VIO base orientation allows us to obtain the LIO/VIO-world
    // to ENU alignment.
    //
    // The resulting transform is then kept fixed because the LIO/VIO
    // world frame is assumed to be static (inertial frame).

    //  Initialize LIO world -> ENU
    // ------------------------------------------------------------
    if ((!gps_topic_.empty()))
    {
        if(!lio_to_enu_.initialized())
        {
            // We need both:
            //   - an initialized INS orientation
            //   - a GPS ENU position
            //
            // The GPS position must come from the GPS callback,
            // NOT from this LIO message.
            if (!orientation_initialized_ || (!enu_converter_.initialized()))
            {
                RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Skipping POSE (LIO/VIO) measurement, missing ENU frame initialization.");
                return;
            }

            // INS body frame expressed w.r.t ENU frame
            auto T_enu_base = initial_enu_base_.isometry();

            // LIO body frame expressed w.r.t INS body frame
            auto T_base_lio = lio_to_base_.isometry();

            // LIO body frame expressed w.r.t LIO world frame
            utils::FrameTransform::Isometry3 
                T_lio_world = Eigen::Isometry3d::Identity();
            T_lio_world.linear() = odom_meas.q.toRotationMatrix();
            T_lio_world.translation() = odom_meas.p;

            // ENU <- LIO world
            auto T_enu_lio_world =
                T_enu_base * T_base_lio * T_lio_world.inverse();

            lio_to_enu_.setTransform(
                T_enu_lio_world.linear(),
                T_enu_lio_world.translation());

            RCLCPP_INFO(
                get_logger(),
                "Initialized LIO/VIO world -> ENU alignment.");
        }

        // Transform LIO pose into ENU frame
        odom_meas =
            lio_to_enu_.transform(odom_meas);
    }else{
        // If GPS is not active, we assume that the LIO/VIO world frame
        // is already aligned with the filter frame. In this case, we only need
        // to transform the LIO/VIO body pose into the INS base frame.
        odom_meas =
            lio_to_base_.transform(odom_meas);
    }

    iESEKF::Group group_meas;
    iESEKF::state_to_group(odom_meas, group_meas);

    // Measurement noise covariance (example values)
    // For odom measurement, we compute 9xDoF matrix
    Eigen::MatrixXd R_odom = Eigen::MatrixXd::Identity(9, 9); 

    if(trust_odom_pose_covariance_)
    {
        Eigen::Matrix<iESEKF::Scalar, 6, 6> pose_cov;
        iESEKF::set_pose_covariance(msg.pose.covariance, pose_cov);
        R_odom.block<3, 3>(0, 0) = pose_cov.block<3, 3>(0, 0); // position covariance
        R_odom.block<3, 3>(6, 6) = pose_cov.block<3, 3>(3, 3); // orientation covariance
    }
    else
    {
        R_odom.block<3, 3>(0, 0) = odom_position_noise_.asDiagonal();    // position covariance
        R_odom.block<3, 3>(6, 6) = odom_orientation_noise_.asDiagonal(); // orientation covariance
    }

    if(trust_odom_velocity_covariance_)
    {
        Eigen::Matrix<iESEKF::Scalar, 3, 3> twist_cov;
        iESEKF::set_velocity_covariance(msg.twist.covariance, twist_cov);
        R_odom.block<3, 3>(3, 3) = twist_cov; // velocity covariance
    }
    else
    {
        R_odom.block<3, 3>(3, 3) = odom_velocity_noise_.asDiagonal(); // velocity covariance
    }

    Eigen::MatrixXd R_odom_inv = R_odom.inverse();

    print_state("Before 3D odometry update", state_);

    filter_.update<iESEKF::Group, iESEKF::Measurement, iESEKF::HMat>(
        group_meas,
        R_odom, R_odom_inv,
        ins_ros::iESEKF::odom::H_fun);

    State state_now;
    const auto noww_system = std::chrono::steady_clock::now();
    const double noww =
        std::chrono::duration<double>(noww_system - t0_system_).count();
    RCLCPP_DEBUG(
        get_logger(),
        "After 3D odometry update, time=%.3f s",
        noww);
    iESEKF::group_to_state(filter_.getState(), noww, state_now);
    print_state("After 3D odometry update", state_now);
    
    nav_msgs::msg::Odometry debug_msg;
    from_ins_to_ros(odom_meas, debug_msg);
    debug_odom_pub_->publish(debug_msg);
}

void INSEstimator::wheel_odom_callback(const geometry_msgs::msg::TwistStamped& msg)
{
    if(!orientation_initialized_)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Orientation not initialized. Skipping wheel odometry measurement.");
        return;
    }

    iESEKF::Measurement meas = iESEKF::Measurement::Zero(3);
    meas(0) = msg.twist.linear.x;
    meas(1) = msg.twist.linear.y;
    meas(2) = 0.0; // Assuming no vertical velocity from wheel odometry

    RCLCPP_DEBUG(get_logger(), "Received base velocity x:%f, y:%f", meas(0), meas(1));

    // Wheel odometry measurement update
    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;
    Mat3 R_w  = Mat3::Zero();
    R_w(0, 0) = wheel_odom_noise_.x();
    R_w(1, 1) = wheel_odom_noise_.y();
    R_w(2, 2) = wheel_odom_noise_.z();
    Mat3 R_w_inv = R_w.inverse();

    filter_.update<iESEKF::Measurement, iESEKF::Measurement, iESEKF::HMat>(
        meas,
        R_w, R_w_inv,
        ins_ros::iESEKF::wheel::H_fun);
    
    State state_now;
    const auto now_system = std::chrono::steady_clock::now();
    const double now =
        std::chrono::duration<double>(now_system - t0_system_).count();
    iESEKF::group_to_state(filter_.getState(), now, state_now);
    print_state("After wheel odometry update", state_now);
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
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////        CONVERSION HELPERS           /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void INSEstimator::from_ros_to_ins(const sensor_msgs::msg::Imu& in, iESEKF::IMUmeas& out)
{
    out.stamp = rclcpp::Time(in.header.stamp).seconds();

    out.gyro(0) = static_cast<iESEKF::Scalar>(in.angular_velocity.x);
    out.gyro(1) = static_cast<iESEKF::Scalar>(in.angular_velocity.y);
    out.gyro(2) = static_cast<iESEKF::Scalar>(in.angular_velocity.z);

    out.accel(0) = static_cast<iESEKF::Scalar>(in.linear_acceleration.x);
    out.accel(1) = static_cast<iESEKF::Scalar>(in.linear_acceleration.y);
    out.accel(2) = static_cast<iESEKF::Scalar>(in.linear_acceleration.z);

    out.bias.gyro = state_.bias.w;
    out.bias.accel = state_.bias.a;
}

void INSEstimator::from_ros_to_ins(const geometry_msgs::msg::PoseStamped& in, ins_ros::State& out)
{
    // Position
    out.p.x() = static_cast<State::Scalar>(in.pose.position.x);
    out.p.y() = static_cast<State::Scalar>(in.pose.position.y);
    out.p.z() = static_cast<State::Scalar>(in.pose.position.z);

    // Orientation
    out.q.x() = static_cast<State::Scalar>(in.pose.orientation.x);
    out.q.y() = static_cast<State::Scalar>(in.pose.orientation.y);
    out.q.z() = static_cast<State::Scalar>(in.pose.orientation.z);
    out.q.w() = static_cast<State::Scalar>(in.pose.orientation.w);
}

void INSEstimator::from_ros_to_ins(const nav_msgs::msg::Odometry& in, ins_ros::State& out)
{
    // Position
    out.p.x() = static_cast<State::Scalar>(in.pose.pose.position.x);
    out.p.y() = static_cast<State::Scalar>(in.pose.pose.position.y);
    out.p.z() = static_cast<State::Scalar>(in.pose.pose.position.z);

    // Orientation
    out.q.x() = static_cast<State::Scalar>(in.pose.pose.orientation.x);
    out.q.y() = static_cast<State::Scalar>(in.pose.pose.orientation.y);
    out.q.z() = static_cast<State::Scalar>(in.pose.pose.orientation.z);
    out.q.w() = static_cast<State::Scalar>(in.pose.pose.orientation.w);

    // Velocity
    out.v.x() = static_cast<State::Scalar>(in.twist.twist.linear.x);
    out.v.y() = static_cast<State::Scalar>(in.twist.twist.linear.y);
    out.v.z() = static_cast<State::Scalar>(in.twist.twist.linear.z);
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

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////        ADDITIONAL HELPERS           /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void INSEstimator::publish_odom()
{
    nav_msgs::msg::Odometry state_msg;
    from_ins_to_ros(state_, state_msg);
    state_pub_->publish(state_msg);
}

void INSEstimator::publish_pose()
{
    geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
    from_ins_to_ros(state_, pose_msg);
    pose_pub_->publish(pose_msg);
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

bool INSEstimator::transform_imu_to_base_link(
    const sensor_msgs::msg::Imu & msg,
    iESEKF::IMUmeas& imu)
{
    if (!imu_to_base_.initialized())
    {
        if (!imu_to_base_.initialize(msg.header.frame_id, body_frame_))
            return false;
    }

    // Angular velocity
    const State::V3 omega_base =
        imu_to_base_.rotate(imu.gyro);

    // Linear acceleration
    const State::V3 accel_base =
        imu_to_base_.rotate(imu.accel);

    State::V3 accel_base_corrected =
        accel_base;

    // Need angular acceleration for the lever-arm correction.
    const double dt = imu.dt;

    if (dt > 0.0 && dt < 0.1)
    {
        const State::V3 alpha_base =
            (omega_base - previous_omega_base_) / dt;

        // a_B = a_I + alpha x r + omega x (omega x r)
        accel_base_corrected +=
            alpha_base.cross(imu_to_base_.translation());

        accel_base_corrected +=
            omega_base.cross(
                omega_base.cross(imu_to_base_.translation()));
    }else{
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            5000,
            "IMU dt is too large or negative (dt=%.6f). Skipping lever-arm correction.",
            dt);
    }

    // Store values for next measurement.
    previous_omega_base_ = omega_base;

    // Output
    imu.gyro = omega_base;
    imu.accel = accel_base_corrected;

    return true;
}

void INSEstimator::initialize_orientation()
{
    if (orientation_initialized_)
        return;

    Eigen::Quaterniond q_tilt = Eigen::Quaterniond::Identity();
    if(estimate_imu_orientation_)
        q_tilt = imu_orientation_initializer_->orientation();
    
    Eigen::Quaterniond q_yaw = Eigen::Quaterniond::Identity();
    if(!gps_topic_.empty())
        q_yaw = gps_orientation_initializer_->orientation();

    /*
     * q_tilt contains roll/pitch w.r.t body/base frame.
     * q_yaw contains GPS-derived yaw (if active).
     */
    Eigen::Quaterniond q_enu_base =
        q_yaw * q_tilt;
    q_enu_base.normalize();

    // Save the initial ENU <-> base_link alignment
    Eigen::Vector3d initial_gps_position_enu = - q_enu_base.toRotationMatrix() * gps_lever_arm_;
    initial_enu_base_.setTransform(q_enu_base, initial_gps_position_enu);

    state_.q =
        q_enu_base.cast<iESEKF::Scalar>();
    state_.p =
        initial_gps_position_enu.cast<iESEKF::Scalar>(); 
    
    setState();
    
    // Update filter with initial orientation measurement
    // iESEKF::Group group_meas;
    // iESEKF::state_to_group(state_, group_meas);
    // Eigen::MatrixXd R_pose = Eigen::MatrixXd::Identity(6, 6) * 0.1; // 10cm/0.1rad covariance
    // Eigen::MatrixXd R_pose_inv = R_pose.inverse();
    // filter_.update<iESEKF::Group, iESEKF::Measurement, iESEKF::HMat>(
    //     group_meas,
    //     R_pose, R_pose_inv,
    //     ins_ros::iESEKF::pose::H_fun);

    orientation_initialized_ = true;

    if(!gps_topic_.empty())
    {
        if(estimate_imu_orientation_)
            RCLCPP_INFO(
                get_logger(),
                "INS orientation initialized taking into account IMU + GPS.");
        else
            RCLCPP_INFO(
                get_logger(),
                "INS orientation initialized taking into account GPS only.");  
    }else{
        if(estimate_imu_orientation_)
            RCLCPP_INFO(
                get_logger(),
                "INS orientation initialized taking into account IMU only.");
        else
            RCLCPP_INFO(
                get_logger(),
                "INS orientation assumed identity at initialization.");
    }

    const auto rpy = state_.get_rpy();

    RCLCPP_INFO(
        get_logger(),
        "Initial RPY: [%.3f, %.3f, %.3f] deg",
        rpy.x(),
        rpy.y(),
        rpy.z());
}

void INSEstimator::print_state(const std::string& prefix, const State& state)
{
    const auto rpy = state.get_rpy();
    const auto v_body = state.get_body_velocity();

    RCLCPP_DEBUG(
        get_logger(),
        "%s:\n"
        "Estimated state:\n"
        "  time:       %.6f\n"
        "  position:   [%.6f, %.6f, %.6f] m\n"
        "  velocity:   [%.6f, %.6f, %.6f] m/s\n"
        "  body velocity:   [%.6f, %.6f, %.6f] m/s\n"
        "  RPY:        [%.3f, %.3f, %.3f] deg\n"
        "  quaternion: [%.6f, %.6f, %.6f, %.6f]\n"
        "  gravity:    [%.6f, %.6f, %.6f] m/s²\n"
        "  angular vel:[%.6f, %.6f, %.6f] rad/s\n"
        "  accel:      [%.6f, %.6f, %.6f] m/s²\n"
        "  gyro bias:  [%.6f, %.6f, %.6f] rad/s\n"
        "  accel bias: [%.6f, %.6f, %.6f] m/s²",
        prefix.c_str(),
        state.time,
        state.p.x(), state.p.y(), state.p.z(),
        state.v.x(), state.v.y(), state.v.z(),
        v_body.x(), v_body.y(), v_body.z(),
        rpy.x(),
        rpy.y(),
        rpy.z(),
        state.q.w(), state.q.x(), state.q.y(), state.q.z(),
        state.g.x(), state.g.y(), state.g.z(),
        state.w.x(), state.w.y(), state.w.z(),
        state.a.x(), state.a.y(), state.a.z(),
        state.bias.w.x(), state.bias.w.y(), state.bias.w.z(),
        state.bias.a.x(), state.bias.a.y(), state.bias.a.z());
}

void INSEstimator::publish_gps_debug(const Eigen::Vector3d& gps_position)
{
    // Publish GPS position in INS frame (body w.r.t ENU)
    auto R = state_.q.toRotationMatrix().cast<double>();
    auto gps_body = gps_position - R * gps_lever_arm_.cast<double>();

    geometry_msgs::msg::Point p;
    p.x = gps_body.x();
    p.y = gps_body.y();
    p.z = gps_body.z();

    debug_gps_points_.push_back(p);

    visualization_msgs::msg::Marker marker;

    marker.header.stamp = this->now();
    marker.header.frame_id = world_frame_;

    marker.ns = "gps_debug";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.orientation.w = 1.0;

    marker.scale.x = 0.05;

    marker.color.a = 1.0;
    marker.color.b = 1.0;

    marker.points = debug_gps_points_;

    debug_gps_pub_->publish(marker);
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

