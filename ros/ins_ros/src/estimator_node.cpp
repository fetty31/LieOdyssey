#include "ins_ros/estimator_node.hpp"

namespace ins_ros {

INSEstimator::INSEstimator(const std::string& node_name)
    : LifecycleNode(node_name)
    , filter_(iESEKF::MatDoF::Identity() * 1e-3,
              iESEKF::Filter::NoiseMatrix::Identity() * 1e-3,
              iESEKF::f,
              iESEKF::df_dx,
              iESEKF::df_dw,
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
    this->filter_time_ = -1.0;
    this->filter_time_initialized_ = false;

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

    // Apply buffer configuration to the measurement handler
    measurements::MeasurementHandler::Options handler_opts;
    handler_opts.imu_capacity = imu_buffer_capacity_;
    handler_opts.aiding_capacity = aiding_buffer_capacity_;
    handler_opts.state_capacity = imu_buffer_capacity_;
    handler_opts.history_window_s = history_window_s_;
    meas_handler_.setOptions(handler_opts);
    meas_handler_.clear();

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

    RCLCPP_INFO(get_logger(), "Configured");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_activate(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Activating...");

    state_pub_->on_activate();
    pose_pub_->on_activate();

    setup_timer();

    RCLCPP_INFO(get_logger(), "Activated (estimation rate: %.2f Hz)", estimation_rate_);
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_deactivate(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Deactivating...");

    estimation_timer_.reset();

    state_pub_->on_deactivate();
    pose_pub_->on_deactivate();

    RCLCPP_INFO(get_logger(), "Deactivated");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_cleanup(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Cleaning up...");

    estimation_timer_.reset();

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
    meas_handler_.clear();
    filter_time_ = -1.0;
    filter_time_initialized_ = false;

    RCLCPP_INFO(get_logger(), "Cleaned up");
    return CallbackReturn::SUCCESS;
}

INSEstimator::CallbackReturn INSEstimator::on_shutdown(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Shutting down...");

    estimation_timer_.reset();

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

    estimation_timer_.reset();

    imu_sub_.reset();
    gps_sub_.reset();
    wheel_odom_sub_.reset();
    odom_sub_.reset();
    mag_sub_.reset();
    baro_sub_.reset();
    state_pub_.reset();
    pose_pub_.reset();
    tf_broadcaster_.reset();

    meas_handler_.clear();

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

    // Timing: when true, stamp incoming measurements at receive time instead
    // of trusting msg.header.stamp (use when sensor clocks are not synced
    // with the PC clock; applies to all sensors including the IMU time base).
    declare_parameter<bool>("timing.use_receive_stamp", false);

    // Filter
    declare_parameter<int>("filter.iterations.max", 5);
    declare_parameter<double>("filter.iterations.tolerance", 1e-6);

    declare_parameter<double>("filter.process_noise.gyro", 6.01e-4);
    declare_parameter<double>("filter.process_noise.accel", 1.53e-2);

    declare_parameter<double>("filter.process_noise.gyro_bias", 1.54e-5);
    declare_parameter<double>("filter.process_noise.accel_bias", 3.38e-4);

    // Fixed-frequency estimation loop
    declare_parameter<double>("filter.rate", 100.0);
    declare_parameter<double>("filter.history_window", 5.0);
    declare_parameter<int>("filter.buffer.imu_capacity", 2000);
    declare_parameter<int>("filter.buffer.aiding_capacity", 200);

    // Synchronization / latency handling
    declare_parameter<double>("sync.gps.rewind_threshold", 0.05);
    declare_parameter<double>("sync.gps.max_age", 2.0);
    declare_parameter<double>("sync.odom.tolerance", 0.05);
    declare_parameter<double>("sync.wheel_odom.tolerance", 0.05);
    declare_parameter<double>("sync.mag.tolerance", 0.05);
    declare_parameter<double>("sync.baro.tolerance", 0.05);
    declare_parameter<double>("sync.yaw.tolerance", 0.10);
    declare_parameter<double>("sync.future_tolerance", 0.02);

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

    use_receive_stamp_ = get_parameter("timing.use_receive_stamp").as_bool();
    if (use_receive_stamp_)
    {
        RCLCPP_WARN(get_logger(),
            "timing.use_receive_stamp=true: stamping all measurements at receive time, ignoring msg headers.");
    }

    // Filter
    max_iters_ = get_parameter("filter.iterations.max").as_int();
    tolerance_ = get_parameter("filter.iterations.tolerance").as_double();

    estimation_rate_ = get_parameter("filter.rate").as_double();
    history_window_s_ = get_parameter("filter.history_window").as_double();
    imu_buffer_capacity_ = static_cast<std::size_t>(get_parameter("filter.buffer.imu_capacity").as_int());
    aiding_buffer_capacity_ = static_cast<std::size_t>(get_parameter("filter.buffer.aiding_capacity").as_int());

    gps_rewind_threshold_ = get_parameter("sync.gps.rewind_threshold").as_double();
    gps_max_age_ = get_parameter("sync.gps.max_age").as_double();
    sync_tolerance_odom_ = get_parameter("sync.odom.tolerance").as_double();
    sync_tolerance_wheel_ = get_parameter("sync.wheel_odom.tolerance").as_double();
    sync_tolerance_mag_ = get_parameter("sync.mag.tolerance").as_double();
    sync_tolerance_baro_ = get_parameter("sync.baro.tolerance").as_double();
    sync_tolerance_yaw_ = get_parameter("sync.yaw.tolerance").as_double();
    sync_future_tolerance_ = get_parameter("sync.future_tolerance").as_double();

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

void INSEstimator::setup_timer()
{
    const double rate = (estimation_rate_ > 0.0) ? estimation_rate_ : 100.0;
    const auto period = std::chrono::duration<double>(1.0 / rate);
    estimation_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&INSEstimator::estimation_timer_callback, this));
    RCLCPP_INFO(get_logger(), "Estimation timer created at %.2f Hz", rate);
}

void INSEstimator::setState() 
{
    iESEKF::Group group;
    iESEKF::state_to_group(this->state_, group);

    this->filter_.setState(group); // set initial state
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////            CALLBACKS (thin)          /////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
 *
 * Each callback converts its ROS message and pushes a stamped measurement
 * into the MeasurementHandler. No predict/update happens here; the
 * fixed-frequency estimation_timer_callback() owns the filter.
 */

void INSEstimator::imu_callback(const sensor_msgs::msg::Imu& msg)
{
    iESEKF::IMUmeas imu;
    from_ros_to_ins(msg, imu);

    // Compute time difference (sensor time base).
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
            "Skipping IMU measurement (TF unavailable).");
        return;
    }

    if (imu.dt < 0.0 || imu.dt >= 0.1)
    {
        // Keep lever-arm correction state consistent, but still buffer the
        // sample so the timer can observe the time gap.
        previous_omega_base_ = imu.gyro;
        if (imu.dt < 0.0)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                "Out-of-order IMU stamp (dt=%.6f). Sample buffered anyway.", imu.dt);
            imu.dt = 0.0;
        }
    }

    // Feed stationary initializer (roll/pitch + bias) while uninitialized.
    if (!imu_orientation_initializer_->initialized() && estimate_imu_orientation_)
    {
        Eigen::Vector3d accel = imu.accel.cast<double>();
        Eigen::Vector3d gyro = imu.gyro.cast<double>();
        if (imu_orientation_initializer_->add_measurement(accel, gyro))
        {
            if (estimate_imu_bias_)
            {
                state_.bias.a = imu_orientation_initializer_->accelerometer_bias().cast<iESEKF::Scalar>();
                state_.bias.w = imu_orientation_initializer_->gyroscope_bias().cast<iESEKF::Scalar>();
            }
            RCLCPP_INFO(get_logger(),
                "IMU orientation computed (roll/pitch) with accel bias: [%.3f, %.3f, %.3f] m/s^2 and gyro bias: [%.3f, %.3f, %.3f] rad/s",
                state_.bias.a.x(), state_.bias.a.y(), state_.bias.a.z(),
                state_.bias.w.x(), state_.bias.w.y(), state_.bias.w.z());
        }
        else
        {
            RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 5000,
                "IMU orientation initializer: %zu samples collected",
                imu_orientation_initializer_->sample_count_);
        }
    }

    meas_handler_.pushImu(imu);
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

    const double stamp = sensor_stamp(msg.header.stamp);

    // Feed yaw initializer (needs motion); kept here so the timer only reads the result.
    if (!gps_orientation_initializer_->initialized())
    {
        if (!gps_orientation_initializer_->add_position(p_gps_enu, stamp))
        {
            RCLCPP_DEBUG(get_logger(),
                "GPS orientation initializer: %f m traveled",
                gps_orientation_initializer_->distance_traveled_);
        }
        else
        {
            RCLCPP_INFO(get_logger(), "GPS orientation initialized (yaw)");
        }
    }

    // Continuous heading refinement -> pushed as yaw measurement for the timer.
    if (gps_orientation_continuous_->add_position(p_gps_enu, stamp))
    {
        measurements::StampedYaw yaw_meas;
        yaw_meas.stamp = stamp;
        yaw_meas.yaw = gps_orientation_continuous_->heading();
        yaw_meas.R.setIdentity();
        yaw_meas.R *= 0.01;
        yaw_meas.R_inv = yaw_meas.R.inverse();
        meas_handler_.pushYaw(yaw_meas);
        publish_yaw_debug(yaw_meas.yaw, p_gps_enu);
        gps_orientation_continuous_->reset();
    }

    measurements::StampedGps meas;
    meas.stamp = stamp;
    meas.meas.position_enu = p_gps_enu.cast<iESEKF::Scalar>();
    meas.meas.lever_arm = gps_lever_arm_.cast<iESEKF::Scalar>();

    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;
    Mat3 R_gps;
    if ((msg.position_covariance_type !=
        sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN) && trust_gps_covariance_)
    {
        R_gps <<
            msg.position_covariance[0], msg.position_covariance[1], msg.position_covariance[2],
            msg.position_covariance[3], msg.position_covariance[4], msg.position_covariance[5],
            msg.position_covariance[6], msg.position_covariance[7], msg.position_covariance[8];
    }
    else
    {
        R_gps = Mat3::Zero();
        R_gps(0, 0) = gps_noise_.x();
        R_gps(1, 1) = gps_noise_.y();
        R_gps(2, 2) = gps_noise_.z();
    }
    meas.R = R_gps;
    meas.R_inv = R_gps.inverse();

    meas_handler_.pushGps(meas);
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

    State odom_meas;
    from_ros_to_ins(msg, odom_meas);
    odom_meas.v = odom_meas.q.toRotationMatrix() * odom_meas.v; // body -> inertial

    const double stamp = sensor_stamp(msg.header.stamp);
    odom_meas.time = stamp;

    if ((!gps_topic_.empty()))
    {
        if(!lio_to_enu_.initialized())
        {
            if (!orientation_initialized_ || (!enu_converter_.initialized()))
            {
                RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000,
                "Skipping POSE (LIO/VIO) measurement, missing ENU frame initialization.");
                return;
            }

            auto T_enu_base = initial_enu_base_.isometry();
            auto T_base_lio = lio_to_base_.isometry();

            utils::FrameTransform::Isometry3
                T_lio_world = Eigen::Isometry3d::Identity();
            T_lio_world.linear() = odom_meas.q.toRotationMatrix();
            T_lio_world.translation() = odom_meas.p;

            auto T_enu_lio_world = T_enu_base * T_base_lio * T_lio_world.inverse();

            lio_to_enu_.setTransform(
                T_enu_lio_world.linear(),
                T_enu_lio_world.translation());

            RCLCPP_INFO(get_logger(), "Initialized LIO/VIO world -> ENU alignment.");
        }

        odom_meas = lio_to_enu_.transform(odom_meas);
    }else{
        odom_meas = lio_to_base_.transform(odom_meas);
    }

    iESEKF::Group group_meas;
    iESEKF::state_to_group(odom_meas, group_meas);

    Eigen::MatrixXd R_odom = Eigen::MatrixXd::Identity(9, 9);
    if(trust_odom_pose_covariance_)
    {
        Eigen::Matrix<iESEKF::Scalar, 6, 6> pose_cov;
        iESEKF::set_pose_covariance(msg.pose.covariance, pose_cov);
        R_odom.block<3, 3>(0, 0) = pose_cov.block<3, 3>(0, 0);
        R_odom.block<3, 3>(6, 6) = pose_cov.block<3, 3>(3, 3);
    }
    else
    {
        R_odom.block<3, 3>(0, 0) = odom_position_noise_.asDiagonal();
        R_odom.block<3, 3>(6, 6) = odom_orientation_noise_.asDiagonal();
    }

    if(trust_odom_velocity_covariance_)
    {
        Eigen::Matrix<iESEKF::Scalar, 3, 3> twist_cov;
        iESEKF::set_velocity_covariance(msg.twist.covariance, twist_cov);
        R_odom.block<3, 3>(3, 3) = twist_cov;
    }
    else
    {
        R_odom.block<3, 3>(3, 3) = odom_velocity_noise_.asDiagonal();
    }

    measurements::StampedOdom stamped;
    stamped.stamp = stamp;
    stamped.group = group_meas;
    stamped.R = R_odom;
    stamped.R_inv = R_odom.inverse();
    meas_handler_.pushOdom(stamped);

    nav_msgs::msg::Odometry debug_msg;
    from_ins_to_ros(odom_meas, debug_msg);
    debug_odom_pub_->publish(debug_msg);
}

void INSEstimator::wheel_odom_callback(const geometry_msgs::msg::TwistStamped& msg)
{
    iESEKF::Measurement meas = iESEKF::Measurement::Zero(3);
    meas(0) = msg.twist.linear.x;
    meas(1) = msg.twist.linear.y;
    meas(2) = 0.0;

    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;
    Mat3 R_w = Mat3::Zero();
    R_w(0, 0) = wheel_odom_noise_.x();
    R_w(1, 1) = wheel_odom_noise_.y();
    R_w(2, 2) = wheel_odom_noise_.z();

    measurements::StampedWheel stamped;
    stamped.stamp = sensor_stamp(msg.header.stamp);
    stamped.meas = meas;
    stamped.R = R_w;
    stamped.R_inv = R_w.inverse();
    meas_handler_.pushWheel(stamped);
}

void INSEstimator::mag_callback(const sensor_msgs::msg::MagneticField& msg)
{
    State::V3 mag_vector;
    mag_vector.x() = msg.magnetic_field.x;
    mag_vector.y() = msg.magnetic_field.y;
    mag_vector.z() = msg.magnetic_field.z;

    measurements::StampedMag stamped;
    stamped.stamp = sensor_stamp(msg.header.stamp);
    stamped.meas = iESEKF::Measurement(mag_vector);
    // R defaults from stamped type; keep fixed variance for now.
    meas_handler_.pushMag(stamped);
}

void INSEstimator::baro_callback(const sensor_msgs::msg::FluidPressure& msg)
{
    measurements::StampedBaro stamped;
    stamped.stamp = sensor_stamp(msg.header.stamp);
    stamped.pressure = static_cast<iESEKF::Scalar>(msg.fluid_pressure);
    stamped.R = Eigen::MatrixXd::Identity(1,1);
    stamped.R(0, 0) = 2.0;
    stamped.R_inv = stamped.R.inverse();
    meas_handler_.pushBaro(stamped);
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////        ESTIMATION TIMER              /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void INSEstimator::estimation_timer_callback()
{
    // Gate 1: ENU frame must exist if GPS is enabled.
    if (!gps_topic_.empty() && !enu_converter_.initialized())
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "Local ENU frame not initialized. Dropping buffered IMU.");
        // Keep buffers bounded while waiting for the first GPS fix.
        const double latest = meas_handler_.latestImuStamp();
        if (latest > 0.0) meas_handler_.drainImuUpTo(latest);
        return;
    }

    // Gate 2: orientation must be initialized before filtering.
    if (!orientation_initialized_)
    {
        if (!try_initialize_orientation())
        {
            const double latest = meas_handler_.latestImuStamp();
            if (latest > 0.0) meas_handler_.drainImuUpTo(latest);
            return;
        }
        // Freshly initialized: anchor filter time to the newest IMU and
        // seed the snapshot history so future GPS rewinds have a reference.
        const double latest = meas_handler_.latestImuStamp();
        if (latest > 0.0)
        {
            meas_handler_.drainImuUpTo(latest);
            filter_time_ = latest;
            filter_time_initialized_ = true;
            meas_handler_.pushStateSnapshot(filter_time_, filter_.getState(), filter_.getCovariance());
            refresh_state_from_filter(filter_time_);
        }
        return;
    }

    const double t_target = meas_handler_.latestImuStamp();
    if (t_target < 0.0) return;
    if (filter_time_initialized_ && t_target <= filter_time_) return;

    // Predict with all new IMU up to the target stamp.
    process_imu_up_to(t_target);
    if (!filter_time_initialized_) return;

    const double filter_time = filter_time_;

    // Update with synchronized aiding measurements.
    process_gps_at(filter_time);
    process_odom_at(filter_time);
    process_wheel_at(filter_time);
    process_mag_at(filter_time);
    process_baro_at(filter_time);
    process_yaw_at(filter_time);

    // Publish current belief.
    publish_odom();
    publish_pose();
    if (publish_tf_) broadcast_tf(state_);

    // Keep history bounded.
    meas_handler_.pruneOlderThan(filter_time - history_window_s_);
}

void INSEstimator::process_imu_up_to(double t_target)
{
    auto imus = meas_handler_.drainImuUpTo(t_target);
    if (imus.empty()) return;

    for (const auto& imu : imus)
    {
        if (imu.dt <= 0.0 || imu.dt >= 0.1)
        {
            // First sample after init or time gap: advance the time base
            // without integrating.
            if (imu.stamp > filter_time_)
            {
                filter_time_ = imu.stamp;
                filter_time_initialized_ = true;
            }
            continue;
        }

        RCLCPP_DEBUG_THROTTLE(get_logger(), *get_clock(), 1000,
            "Predict: accel=[%.3f, %.3f, %.3f] gyro=[%.3f, %.3f, %.3f] dt=%.6f",
            imu.accel.x(), imu.accel.y(), imu.accel.z(),
            imu.gyro.x(), imu.gyro.y(), imu.gyro.z(), imu.dt);

        filter_.predict(imu);
        filter_time_ = imu.stamp;
        filter_time_initialized_ = true;

        iESEKF::group_to_state(filter_.getState(), filter_time_, state_);
        state_.w = imu.gyro;
        state_.a = imu.accel;

        meas_handler_.pushStateSnapshot(filter_time_, filter_.getState(), filter_.getCovariance());
    }
}

void INSEstimator::process_gps_at(double filter_time)
{
    if (gps_topic_.empty()) return;

    auto opt = meas_handler_.takeGpsAtOrBefore(filter_time);
    if (!opt) return;

    const double delay = filter_time - opt->stamp;
    if (delay > gps_max_age_)
    {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Dropping stale GPS (delay=%.3f s > max_age=%.3f s)", delay, gps_max_age_);
        return;
    }

    if (measurements::MeasurementHandler::needsRewind(opt->stamp, filter_time, gps_rewind_threshold_))
    {
        if (apply_gps_with_rewind(*opt, filter_time)) return;
        // Fall through to direct update if rewind history is unavailable.
    }

    apply_gps_direct(*opt);
}

bool INSEstimator::apply_gps_with_rewind(const measurements::StampedGps& gps, double filter_time)
{
    auto snap = meas_handler_.snapshotAt(gps.stamp);
    if (!snap)
    {
        RCLCPP_WARN(get_logger(), "No state snapshot for GPS rewind at %.6f; applying direct update.", gps.stamp);
        return false;
    }
    auto imus = meas_handler_.imuBetween(gps.stamp, filter_time);
    if (imus.empty())
    {
        RCLCPP_DEBUG(get_logger(), "No IMU between GPS stamp and filter time; applying direct update.");
        return false;
    }

    RCLCPP_DEBUG(get_logger(), "GPS delayed by %.3f s: rewind to %.6f and re-propagate %zu IMU.",
        filter_time - gps.stamp, gps.stamp, imus.size());

    // Save current belief.
    const auto X_cur = filter_.getState();
    const auto P_cur = filter_.getCovariance();
    (void)X_cur; (void)P_cur;  // kept for future re-application of in-window aiding

    // Rewind, update at measurement time, then re-propagate (past -> future recovery).
    filter_.setState(snap->state);
    filter_.setCovariance(snap->covariance);

    filter_.update<
        iESEKF::gps::GPSMeasurement,
        iESEKF::Measurement,
        iESEKF::HMat>(gps.meas, gps.R, gps.R_inv, ins_ros::iESEKF::gps::H_fun);

    meas_handler_.truncateSnapshotsAfter(gps.stamp);
    meas_handler_.pushStateSnapshot(gps.stamp, filter_.getState(), filter_.getCovariance());

    for (const auto& imu : imus)
    {
        if (imu.dt <= 0.0 || imu.dt >= 0.1) continue;
        filter_.predict(imu);
        meas_handler_.pushStateSnapshot(imu.stamp, filter_.getState(), filter_.getCovariance());
    }

    refresh_state_from_filter(filter_time);
    print_state("After delayed GPS update + re-propagation", state_);
    return true;
}

void INSEstimator::apply_gps_direct(const measurements::StampedGps& gps)
{
    filter_.update<
        iESEKF::gps::GPSMeasurement,
        iESEKF::Measurement,
        iESEKF::HMat>(gps.meas, gps.R, gps.R_inv, ins_ros::iESEKF::gps::H_fun);

    refresh_state_from_filter(filter_time_);
    meas_handler_.pushStateSnapshot(filter_time_, filter_.getState(), filter_.getCovariance());
    print_state("After GPS update", state_);
}

void INSEstimator::process_odom_at(double filter_time)
{
    if (odom_topic_.empty()) return;
    auto opt = meas_handler_.takeSyncOdom(filter_time, sync_tolerance_odom_, sync_future_tolerance_);
    if (!opt) return;

    print_state("Before 3D odometry update", state_);
    filter_.update<iESEKF::Group, iESEKF::Measurement, iESEKF::HMat>(
        opt->group, opt->R, opt->R_inv, ins_ros::iESEKF::odom::H_fun);
    refresh_state_from_filter(filter_time);
    meas_handler_.pushStateSnapshot(filter_time, filter_.getState(), filter_.getCovariance());
    print_state("After 3D odometry update", state_);
}

void INSEstimator::process_wheel_at(double filter_time)
{
    if (wheel_odom_topic_.empty()) return;
    auto opt = meas_handler_.takeSyncWheel(filter_time, sync_tolerance_wheel_, sync_future_tolerance_);
    if (!opt) return;

    filter_.update<iESEKF::Measurement, iESEKF::Measurement, iESEKF::HMat>(
        opt->meas, opt->R, opt->R_inv, ins_ros::iESEKF::wheel::H_fun);
    refresh_state_from_filter(filter_time);
    meas_handler_.pushStateSnapshot(filter_time, filter_.getState(), filter_.getCovariance());
    print_state("After wheel odometry update", state_);
}

void INSEstimator::process_mag_at(double filter_time)
{
    if (mag_topic_.empty()) return;
    auto opt = meas_handler_.takeSyncMag(filter_time, sync_tolerance_mag_, sync_future_tolerance_);
    if (!opt) return;

    filter_.update<iESEKF::Measurement, iESEKF::Measurement, iESEKF::HMat>(
        opt->meas, opt->R, opt->R_inv, ins_ros::iESEKF::magnetometer::H_fun);
    refresh_state_from_filter(filter_time);
    meas_handler_.pushStateSnapshot(filter_time, filter_.getState(), filter_.getCovariance());
}

void INSEstimator::process_baro_at(double filter_time)
{
    if (baro_topic_.empty()) return;
    auto opt = meas_handler_.takeSyncBaro(filter_time, sync_tolerance_baro_, sync_future_tolerance_);
    if (!opt) return;

    filter_.update<iESEKF::Scalar, iESEKF::Measurement, iESEKF::HMat>(
        opt->pressure, opt->R, opt->R_inv, ins_ros::iESEKF::barometer::H_fun);
    refresh_state_from_filter(filter_time);
    meas_handler_.pushStateSnapshot(filter_time, filter_.getState(), filter_.getCovariance());
}

void INSEstimator::process_yaw_at(double filter_time)
{
    if (gps_topic_.empty()) return;
    auto opt = meas_handler_.takeSyncYaw(filter_time, sync_tolerance_yaw_, sync_future_tolerance_);
    if (!opt) return;

    filter_.update<
        iESEKF::Scalar,
        iESEKF::Measurement,
        iESEKF::HMat>(
            opt->yaw, opt->R, opt->R_inv, ins_ros::iESEKF::yaw::H_fun);
    RCLCPP_DEBUG(get_logger(), "Updating yaw: %f deg", opt->yaw * 180.0 / M_PI);
    refresh_state_from_filter(filter_time);
    meas_handler_.pushStateSnapshot(filter_time, filter_.getState(), filter_.getCovariance());
    print_state("After yaw update", state_);
}

void INSEstimator::refresh_state_from_filter(double stamp)
{
    iESEKF::group_to_state(filter_.getState(), stamp, state_);
}

bool INSEstimator::try_initialize_orientation()
{
    if (orientation_initialized_) return true;
    if (!imu_orientation_initializer_->initialized()) return false;
    if ((!gps_orientation_initializer_->initialized()) && (!gps_topic_.empty())) return false;

    initialize_orientation();
    return orientation_initialized_;
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////        CONVERSION HELPERS           /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

double INSEstimator::sensor_stamp(const rclcpp::Time& header_stamp)
{
    if (use_receive_stamp_)
        return this->get_clock()->now().seconds();
    return header_stamp.seconds();
}

void INSEstimator::from_ros_to_ins(const sensor_msgs::msg::Imu& in, iESEKF::IMUmeas& out)
{
    out.stamp = sensor_stamp(in.header.stamp);

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
    out.header.stamp = rclcpp::Time(static_cast<int64_t>(in.time * 1e9));
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
    out.header.stamp = rclcpp::Time(static_cast<int64_t>(in.time * 1e9));
    out.header.frame_id = world_frame_;

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
    tf_msg.header.stamp    = (now) ? this->get_clock()->now() : rclcpp::Time(static_cast<int64_t>(in.time * 1e9));
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

    // Anchor the filter to where the antenna actually is now.
    //
    // When GPS is active, heading initialization requires the robot to travel
    // (distance_threshold, default ~2 m) away from the ENU origin, which was
    // fixed at the very first GPS fix. Anchoring at the origin would ignore
    // that traveled distance, so the newest buffered GPS fix is used instead:
    //
    // p_body_enu = p_antenna_enu - R_enu_base * lever_arm
    //
    // A rough initial velocity is differenced from the two newest buffered
    // fixes (horizontal plane only; GPS altitude is too noisy for this) and
    // the anchored position is extrapolated to the latest IMU stamp, which is
    // the time base the timer will continue filtering from.
    Eigen::Vector3d p_antenna_enu = Eigen::Vector3d::Zero();
    double t_antenna = -1.0;
    Eigen::Vector3d v_init_enu = Eigen::Vector3d::Zero();
    bool have_anchor = false;

    if (!gps_topic_.empty())
    {
        const auto newest = meas_handler_.peekNewestGps(2);
        if (!newest.empty())
        {
            p_antenna_enu = newest.back().meas.position_enu.cast<double>();
            t_antenna = newest.back().stamp;
            have_anchor = true;

            if (newest.size() == 2)
            {
                const double dt = newest[1].stamp - newest[0].stamp;
                const Eigen::Vector2d dp =
                    (newest[1].meas.position_enu - newest[0].meas.position_enu)
                        .head<2>().cast<double>();
                const double speed = (dt > 1e-6) ? dp.norm() / dt : -1.0;

                // Sanity gate: reject non-monotonic stamps and unphysical jumps.
                constexpr double kMaxInitSpeed = 30.0;  // m/s
                if (dt > 1e-6 && speed >= 0.0 && speed <= kMaxInitSpeed)
                {
                    v_init_enu.head<2>() = dp / dt;
                }
                else
                {
                    RCLCPP_WARN(get_logger(),
                        "Discarding GPS velocity seed (dt=%.3f s, speed=%.2f m/s); starting at zero velocity.",
                        dt, speed);
                }
            }
        }
        if (!have_anchor)
        {
            RCLCPP_WARN(get_logger(),
                "No buffered GPS fix at initialization; anchoring filter at ENU origin with zero velocity.");
        }
    }

    const Eigen::Matrix3d R_enu_base = q_enu_base.toRotationMatrix();
    Eigen::Vector3d initial_body_position_enu =
        p_antenna_enu - R_enu_base * gps_lever_arm_;

    // Reference time: newest IMU stamp, matching what the timer anchors
    // filter_time_ to right after initialization.
    double t_ref = meas_handler_.latestImuStamp();
    if (have_anchor && t_ref > 0.0 && t_antenna > 0.0 && t_ref >= t_antenna)
    {
        initial_body_position_enu += v_init_enu * (t_ref - t_antenna);
    }
    else if (t_ref < 0.0)
    {
        t_ref = (t_antenna > 0.0) ? t_antenna : 0.0;
    }

    state_.time = t_ref;

    // Save the initial ENU <-> base_link alignment (base pose at init time,
    // used e.g. to align the LIO/VIO world frame in odom_callback).
    initial_enu_base_.setTransform(q_enu_base, initial_body_position_enu);

    state_.q =
        q_enu_base.cast<iESEKF::Scalar>();
    state_.p =
        initial_body_position_enu.cast<iESEKF::Scalar>();
    state_.v =
        v_init_enu.cast<iESEKF::Scalar>(); 
    
    setState();
    
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
        "Initial RPY: [%.3f, %.3f, %.3f] deg, position: [%.3f, %.3f, %.3f] m, velocity: [%.3f, %.3f, %.3f] m/s (antenna fix: [%.3f, %.3f, %.3f] m @ %.3f s, ref t=%.3f s)",
        rpy.x(),
        rpy.y(),
        rpy.z(),
        state_.p.x(),
        state_.p.y(),
        state_.p.z(),
        state_.v.x(),
        state_.v.y(),
        state_.v.z(),
        p_antenna_enu.x(),
        p_antenna_enu.y(),
        p_antenna_enu.z(),
        t_antenna,
        t_ref);
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

void INSEstimator::publish_yaw_debug(double yaw, const Eigen::Vector3d& position_enu)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = world_frame_;
    marker.header.stamp = this->now();
    marker.ns = "yaw_marker";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = position_enu.x();
    marker.pose.position.y = position_enu.y();
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
