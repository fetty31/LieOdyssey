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
    , wheel_to_base_(tf_buffer_, get_logger())
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
    this->filter_init_time_ = -1.0;
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
    wheel_to_base_.reset();
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
    handler_opts.measurement_capacity = measurement_capacity_;
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
    filter_init_time_ = -1.0;
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

    // Timing: when true, incoming measurements are stamped at receive time instead
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
    declare_parameter<int>("filter.buffer.measurement_capacity", 200);

    // Synchronization / latency handling
    declare_parameter<double>("sync.gps.tolerance", 0.005);
    declare_parameter<double>("sync.odom.tolerance", 0.005);
    declare_parameter<double>("sync.wheel_odom.tolerance", 0.005);
    declare_parameter<double>("sync.mag.tolerance", 0.005);
    declare_parameter<double>("sync.baro.tolerance", 0.005);
    declare_parameter<double>("sync.yaw.tolerance", 0.005);
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
    measurement_capacity_ = static_cast<std::size_t>(get_parameter("filter.buffer.measurement_capacity").as_int());

    sync_tolerance_gps_ = get_parameter("sync.gps.tolerance").as_double();
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

    // static const double MAX_DT = 0.1;

    // if (imu.dt < 0.0) // Timestamp went backwards.
    // {
    //     handleTimeReset(t);
    //     imu.dt = 0.0;
    // }
    // else if(imu.dt > MAX_DT) // Large jump
    // {
    //     handleTimeJump(t, dt);
    //     imu.dt = 0.0;
    // }

    // Feed stationary initializer (roll/pitch + bias) while uninitialized.
    if (!imu_orientation_initializer_->initialized() && estimate_imu_orientation_)
    {
        Eigen::Vector3d accel = imu.accel.cast<double>();
        Eigen::Vector3d gyro = imu.gyro.cast<double>();
        if (imu_orientation_initializer_->add_measurement(accel, gyro, imu.stamp))
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

    meas_handler_.push(imu);
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

    const double stamp = stampToSec(msg.header.stamp);

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
        meas_handler_.push(yaw_meas);
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

    meas_handler_.push(meas);
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
                "Skipping ODOMETRY (LIO/VIO) measurement, missing ENU frame initialization.");
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

    Eigen::MatrixXd R_odom = Eigen::MatrixXd::Identity(10, 10);
    R_odom(9,9) = 0.1; // time sensitivity
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
    stamped.stamp = odom_meas.time;
    stamped.group = group_meas;
    stamped.R = R_odom;
    stamped.R_inv = R_odom.inverse();
    meas_handler_.push(stamped);

    nav_msgs::msg::Odometry debug_msg;
    from_ins_to_ros(odom_meas, debug_msg, msg.pose.covariance, msg.twist.covariance);
    debug_odom_pub_->publish(debug_msg);
}

void INSEstimator::wheel_odom_callback(const geometry_msgs::msg::TwistStamped& msg)
{
    if (!wheel_to_base_.initialized())
    {
        if (!wheel_to_base_.initialize(msg.header.frame_id, body_frame_))
            return;
    }

    Eigen::Vector3d wheel_odom;
    wheel_odom(0) = msg.twist.linear.x;
    wheel_odom(1) = msg.twist.linear.y;
    wheel_odom(2) = 0.0;

    auto base_odom = wheel_to_base_.transform(wheel_odom);

    iESEKF::Measurement meas = iESEKF::Measurement::Zero(3);
    meas(0) = base_odom(0);
    meas(1) = base_odom(1);
    meas(2) = base_odom(2);

    using Mat3 = Eigen::Matrix<iESEKF::Scalar, 3, 3>;
    Mat3 R_w = Mat3::Zero();
    R_w(0, 0) = wheel_odom_noise_.x();
    R_w(1, 1) = wheel_odom_noise_.y();
    R_w(2, 2) = wheel_odom_noise_.z();

    measurements::StampedWheel stamped;
    stamped.stamp = stampToSec(msg.header.stamp);
    stamped.meas = meas;
    stamped.R = R_w;
    stamped.R_inv = R_w.inverse();
    meas_handler_.push(stamped);
}

void INSEstimator::mag_callback(const sensor_msgs::msg::MagneticField& msg)
{
    State::V3 mag_vector;
    mag_vector.x() = msg.magnetic_field.x;
    mag_vector.y() = msg.magnetic_field.y;
    mag_vector.z() = msg.magnetic_field.z;

    measurements::StampedMag stamped;
    stamped.stamp = stampToSec(msg.header.stamp);
    stamped.meas = iESEKF::Measurement(mag_vector);
    // R defaults from stamped type; keep fixed variance for now.
    meas_handler_.push(stamped);
}

void INSEstimator::baro_callback(const sensor_msgs::msg::FluidPressure& msg)
{
    measurements::StampedBaro stamped;
    stamped.stamp = stampToSec(msg.header.stamp);
    stamped.pressure = static_cast<iESEKF::Scalar>(msg.fluid_pressure);
    stamped.R = Eigen::MatrixXd::Identity(1,1);
    stamped.R(0, 0) = 2.0;
    stamped.R_inv = stamped.R.inverse();
    meas_handler_.push(stamped);
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////        ESTIMATION MAIN LOOP          /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

void INSEstimator::estimation_timer_callback()
{
    // Gate 1: ENU frame must exist if GPS is enabled.
    if (!gps_topic_.empty() && !enu_converter_.initialized())
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Local ENU frame not initialized. Waiting for ENU frame.");
        return;
    }

    // Gate 2: ensure filter time reference is set
    if (!filter_time_initialized_)
        return;

    // Gate 3: orientation must be initialized before filtering.
    if (!orientation_initialized_)
    {
        if (!try_initialize_orientation())
            return;

        filter_time_ = state_.time;

        meas_handler_.pushStateSnapshot(
            filter_time_,
            filter_.getState(),
            filter_.getCovariance());

        refresh_state_from_filter();
        return;
    }

    // Process measurements in chronological order.
    while (meas_handler_.hasMeasurements())
    {
        RCLCPP_DEBUG(
            get_logger(),
            "Queue: %zu total | IMU: %zu | GPS: %zu | ODOM: %zu | "
            "WHEEL: %zu | MAG: %zu | BARO: %zu | YAW: %zu",
            meas_handler_.queuedCount(),
            meas_handler_.queuedCountOfType<iESEKF::IMUmeas>(),
            meas_handler_.queuedCountOfType<measurements::StampedGps>(),
            meas_handler_.queuedCountOfType<measurements::StampedOdom>(),
            meas_handler_.queuedCountOfType<measurements::StampedWheel>(),
            meas_handler_.queuedCountOfType<measurements::StampedMag>(),
            meas_handler_.queuedCountOfType<measurements::StampedBaro>(),
            meas_handler_.queuedCountOfType<measurements::StampedYaw>());

        auto measurement = meas_handler_.pop();
        if (!measurement)
            break;

        const double t = measurements::MeasurementHandler::getStamp(*measurement);

        RCLCPP_DEBUG(get_logger(), "Measurement processing at time: %.7f"
                                    " (filter time %.7f)", 
                                    t, filter_time_);

        // OOSM: Out-Of-Sequence Measurement
        // if (t < filter_time_)
        if (false)
        {
            RCLCPP_DEBUG(
                get_logger(),
                "Detected OOSM at %.7f, filter time %.7f",
                t,
                filter_time_);

            const bool success = handleOOSM(*measurement);
            
            // The delayed measurement is now either incorporated or dropped.
            if (!success)
            {
                RCLCPP_WARN(
                    get_logger(),
                    "OOSM processing failed for measurement at %.7f",
                    t);
            }else{
                //
                // handleOOSM() succeeded, so the delayed measurement is marked as processed
                meas_handler_.markProcessed(*measurement);

            }

            continue;
        }

        // Normal chronological processing
        std::visit(
            [this](const auto& m)
            {
                processMeasurement(m);
            },
            measurement->measurement
        );

        // Store it for potential future OOSM replay.
        meas_handler_.markProcessed(*measurement);
    }

    // Publish current belief.
    publish_odom();
    publish_pose();

    if (publish_tf_)
        broadcast_tf(state_);

    // Keep history bounded.
    meas_handler_.pruneOlderThan(
        filter_time_ - history_window_s_);
}

void INSEstimator::processMeasurement(const iESEKF::IMUmeas& imu)
{
    RCLCPP_DEBUG(get_logger(), "Propagating IMU");

    if (imu.stamp <= filter_time_)
        return;

    filter_.predict(imu);

    refresh_state_from_filter();

    meas_handler_.pushStateSnapshot(
        filter_time_,
        filter_.getState(),
        filter_.getCovariance());
}

bool INSEstimator::propagateTo(double t)
{
    RCLCPP_DEBUG(
        get_logger(),
        "propagateTo %.7f from %.7f",
        t,
        filter_time_);

    if (t <= filter_time_)
        return true;

    // Propagate through all complete IMU samples before t.
    const auto imu_samples =
        meas_handler_.imuBetween(filter_time_, t);

    for (const auto& imu : imu_samples)
    {
        if (imu.stamp <= filter_time_)
            continue;

        // Do not consume the sample if it is at/after the target.
        // Its input will be interpolated below.
        if (imu.stamp >= t)
            break;

        if (imu.dt <= 0.0 || imu.dt >= 0.1)
        {
            RCLCPP_WARN(
                get_logger(),
                "Skipping IMU at %.7f with invalid dt %.7f",
                imu.stamp,
                imu.dt);

            continue;
        }

        RCLCPP_DEBUG(
            get_logger(),
            "Propagating complete IMU to %.7f",
            imu.stamp);

        filter_.predict(imu);

        refresh_state_from_filter();
    }

    // If we haven't reached t, create an interpolated IMU sample
    //    exactly at t.
    if (filter_time_ < t)
    {
        auto interpolated =
            meas_handler_.interpolateImuAt(t);

        if (!interpolated)
        {
            RCLCPP_WARN(
                get_logger(),
                "Cannot propagate from %.7f to %.7f: "
                "no IMU samples bracketing target time",
                filter_time_,
                t);

            return false;
        }

        interpolated->dt = t - filter_time_;

        if (interpolated->dt <= 0.0 ||
            interpolated->dt >= 0.1)
        {
            RCLCPP_WARN(
                get_logger(),
                "Invalid interpolated IMU dt %.7f",
                interpolated->dt);

            return false;
        }

        RCLCPP_DEBUG(
            get_logger(),
            "Propagating interpolated IMU to %.7f "
            "(dt %.7f)",
            t,
            interpolated->dt);

        filter_.predict(*interpolated);

        refresh_state_from_filter();
    }

    return std::abs(filter_time_ - t) < 1e-9;
}

void INSEstimator::processMeasurement(const measurements::StampedGps& gps)
{
    RCLCPP_DEBUG(get_logger(), "Updating with GPS: [%.3f, %.3f, %.3f]",
                                gps.meas.position_enu.x(),
                                gps.meas.position_enu.y(),
                                gps.meas.position_enu.z());

    if (gps.stamp > (filter_time_ + sync_tolerance_gps_))
    {
        if (!propagateTo(gps.stamp))
            return;
    }

    process_gps(gps);

    refresh_state_from_filter();

    meas_handler_.pushStateSnapshot(
        filter_time_,
        filter_.getState(),
        filter_.getCovariance());
}

void INSEstimator::processMeasurement(const measurements::StampedOdom& odom)
{
    RCLCPP_DEBUG(get_logger(), "Updating with FULL Odometry");

    if (odom.stamp > (filter_time_ + sync_tolerance_odom_))
    {
        if (!propagateTo(odom.stamp))
            return;
    }

    print_state("Before odometry update", state_);

    // If GPS active --> process relative odom in order to avoid frame alignment
    if (!gps_topic_.empty()){
        process_relative_odom(odom);
    } 
    else
    {
        process_odom(odom);
    }

    refresh_state_from_filter();

    print_state("After odometry update", state_);

    meas_handler_.pushStateSnapshot(
        filter_time_,
        filter_.getState(),
        filter_.getCovariance());
}

void INSEstimator::processMeasurement(const measurements::StampedWheel& wheel)
{
    RCLCPP_DEBUG(get_logger(), "Updating with Wheel Odom: [%.3f, %.3f]", 
                    wheel.meas.x(), 
                    wheel.meas.y()
                );

    if (wheel.stamp > (filter_time_ + sync_tolerance_wheel_))
    {
        if (!propagateTo(wheel.stamp))
            return;
    }

    process_wheel(wheel);

    refresh_state_from_filter();

    meas_handler_.pushStateSnapshot(
        filter_time_,
        filter_.getState(),
        filter_.getCovariance());
}

void INSEstimator::processMeasurement(const measurements::StampedMag& mag)
{
    RCLCPP_DEBUG(get_logger(), "Updating with Mag: [%.3f, %.3f, %.3f]", 
                                mag.meas.x(),
                                mag.meas.y(),
                                mag.meas.z());

    if (mag.stamp > (filter_time_ + sync_tolerance_mag_))
    {
        if (!propagateTo(mag.stamp))
            return;
    }

    process_mag(mag);

    refresh_state_from_filter();

    meas_handler_.pushStateSnapshot(
        filter_time_,
        filter_.getState(),
        filter_.getCovariance());
}

void INSEstimator::processMeasurement(const measurements::StampedBaro& baro)
{
    RCLCPP_DEBUG(get_logger(), "Updating with Baro: %.3f", baro.pressure);

    if (baro.stamp > (filter_time_ + sync_tolerance_baro_))
    {
        if (!propagateTo(baro.stamp))
            return;
    }

    process_baro(baro);

    refresh_state_from_filter();

    meas_handler_.pushStateSnapshot(
        filter_time_,
        filter_.getState(),
        filter_.getCovariance());
}

void INSEstimator::processMeasurement(const measurements::StampedYaw& yaw)
{
    RCLCPP_DEBUG(get_logger(), "Updating with Yaw: %.3f", yaw.yaw);

    if (yaw.stamp > (filter_time_ + sync_tolerance_yaw_))
    {
        if (!propagateTo(yaw.stamp))
            return;
    }

    process_yaw(yaw);

    refresh_state_from_filter();

    meas_handler_.pushStateSnapshot(
        filter_time_,
        filter_.getState(),
        filter_.getCovariance());
}

bool INSEstimator::handleOOSM(
    const measurements::QueuedMeasurement& oosm)
{
    const double t_oosm =
        measurements::MeasurementHandler::getStamp(oosm);

    const double t_current = filter_time_;

    RCLCPP_WARN(
        get_logger(),
        "Handling OOSM at %.6f, current filter time %.6f "
        "(delay %.3f s)",
        t_oosm,
        t_current,
        t_current - t_oosm);

    // Make sure the delayed measurement is still inside
    // the available fixed-lag history.
    if (t_current - t_oosm > history_window_s_)
    {
        RCLCPP_WARN(
            get_logger(),
            "OOSM is %.3f s old, exceeding history window %.3f s. "
            "Dropping measurement.",
            t_current - t_oosm,
            history_window_s_);

        return false;
    }

    // Find the latest state snapshot at or before the OOSM.
    const auto snapshot = meas_handler_.snapshotAt(t_oosm);
    if (!snapshot)
    {
        RCLCPP_WARN(
            get_logger(),
            "No state snapshot available before OOSM at %.6f. "
            "Dropping measurement.",
            t_oosm);

        return false;
    }

    const double t_rewind = snapshot->stamp;

    RCLCPP_DEBUG(
        get_logger(),
        "Rewinding from %.6f to %.6f",
        t_current,
        t_rewind);

    // Collect everything that needs to be replayed.
    auto replay =
        meas_handler_.processedBetween(t_rewind, t_current);

    // Add the delayed measurement itself.
    replay.push_back(oosm);

    // Chronological ordering.
    //
    // stable_sort is intentional: if two measurements have exactly
    // the same timestamp, preserve their existing relative order.
    std::stable_sort(
        replay.begin(),
        replay.end(),
        [](const auto& a, const auto& b)
        {
            return measurements::MeasurementHandler::getStamp(a) <
                   measurements::MeasurementHandler::getStamp(b);
        });

    // Restore filter state/covariance.
    iESEKF::group_to_state(snapshot->group, state_);
    state_.time = t_rewind;
    setState();
    filter_.setCovariance(snapshot->covariance);

    filter_time_ = state_.time;

    // Delete the invalid state-history tail.
    meas_handler_.eraseStateHistoryAfter(t_rewind);

    // Replay everything chronologically.
    for (const auto& queued : replay)
    {
        std::visit(
            [this](const auto& measurement)
            {
                processMeasurement(measurement);
            },
            queued.measurement);
    }

    // Make sure we ended at the same time as before the rewind.
    if (filter_time_ < t_current)
    {
        RCLCPP_DEBUG(
            get_logger(),
            "Replay ended at %.6f, original time was %.6f. "
            "Propagating to original filter time.",
            filter_time_,
            t_current);

        if (!propagateTo(t_current))
        {
            RCLCPP_WARN(
                get_logger(),
                "Could not propagate replayed state back to %.6f",
                t_current);

            return false;
        }

        refresh_state_from_filter();

        meas_handler_.pushStateSnapshot(
            filter_time_,
            filter_.getState(),
            filter_.getCovariance());
    }

    RCLCPP_DEBUG(
        get_logger(),
        "OOSM replay completed. Filter time: %.6f",
        filter_time_);

    return true;
}

void INSEstimator::process_gps(const measurements::StampedGps& gps)
{
    filter_.update<
        iESEKF::gps::GPSMeasurement,
        iESEKF::Measurement,
        iESEKF::HMat>(gps.meas, gps.R, gps.R_inv, ins_ros::iESEKF::gps::H_fun);
}

void INSEstimator::process_odom(const measurements::StampedOdom& odom)
{
    filter_.update<iESEKF::Group, iESEKF::Measurement, iESEKF::HMat>(
        odom.group, odom.R, odom.R_inv, ins_ros::iESEKF::odom::H_fun);
}

void INSEstimator::process_relative_odom(const measurements::StampedOdom& odom)
{
    RCLCPP_DEBUG(
        get_logger(),
        "Process relative ODOMETRY at %.9f",
        odom.stamp);

    // Previous processed odometry = reference.
    auto ref_odom = meas_handler_.peekLatestProcessedOdom();

    if (!ref_odom)
    {
        RCLCPP_DEBUG(
            get_logger(),
            "No previous processed odometry available.");
        return;
    }

    auto snapshot =
        meas_handler_.snapshotAt(ref_odom->stamp);

    if (!snapshot)
    {
        RCLCPP_WARN(
            get_logger(),
            "Could not find state snapshot at %.9f",
            ref_odom->stamp);
        return;
    }

    iESEKF::relative_odom::RelativeOdomMeasurement rel;
    rel.X_ref = snapshot->group;

    rel.Y_ref = ref_odom->group;
    rel.Y_cur = odom.group;

    rel.t_ref = ref_odom->stamp;
    rel.t_cur = odom.stamp;

    filter_.update<
        iESEKF::relative_odom::RelativeOdomMeasurement,
        iESEKF::Measurement,
        iESEKF::HMat>(
        rel,
        odom.R,
        odom.R_inv,
        ins_ros::iESEKF::relative_odom::H_fun);
}

void INSEstimator::process_wheel(const measurements::StampedWheel& wheel)
{
    filter_.update<iESEKF::Measurement, iESEKF::Measurement, iESEKF::HMat>(
        wheel.meas, wheel.R, wheel.R_inv, ins_ros::iESEKF::wheel::H_fun);
}

void INSEstimator::process_mag(const measurements::StampedMag& mag)
{
    filter_.update<iESEKF::Measurement, iESEKF::Measurement, iESEKF::HMat>(
        mag.meas, mag.R, mag.R_inv, ins_ros::iESEKF::magnetometer::H_fun);
}

void INSEstimator::process_baro(const measurements::StampedBaro& baro)
{
    filter_.update<iESEKF::Scalar, iESEKF::Measurement, iESEKF::HMat>(
        baro.pressure, baro.R, baro.R_inv, ins_ros::iESEKF::barometer::H_fun);
}

void INSEstimator::process_yaw(const measurements::StampedYaw& yaw)
{
    filter_.update<iESEKF::Scalar, iESEKF::Measurement, iESEKF::HMat>(
        yaw.yaw, yaw.R, yaw.R_inv, ins_ros::iESEKF::yaw::H_fun);
}

void INSEstimator::refresh_state_from_filter()
{
    iESEKF::group_to_state(filter_.getState(), state_);
    filter_time_ = state_.time;
}

bool INSEstimator::try_initialize_orientation()
{
    if (orientation_initialized_) return true;
    if (!imu_orientation_initializer_->initialized()) return false;
    if ((!gps_orientation_initializer_->initialized()) && (!gps_topic_.empty())) return false;

    initialize_orientation();
    return orientation_initialized_;
}

void INSEstimator::initialize_orientation()
{
    if (orientation_initialized_)
        return;

    // Retrieve roll/pitch from IMU sensor
    Eigen::Quaterniond q_tilt = Eigen::Quaterniond::Identity();
    double t_imu = -1.0;
    if(estimate_imu_orientation_){
        q_tilt = imu_orientation_initializer_->orientation();
        t_imu = imu_orientation_initializer_->stamp();
    }
    
    // Retrieve yaw, position and velocity from GPS
    Eigen::Quaterniond q_yaw = Eigen::Quaterniond::Identity();
    Eigen::Vector2d v_antenna_enu = Eigen::Vector2d::Zero();
    Eigen::Vector2d p_antenna_enu = Eigen::Vector2d::Zero();
    double t_antenna = -1.0;
    if(!gps_topic_.empty())
    {
        q_yaw = gps_orientation_initializer_->orientation();
        v_antenna_enu = gps_orientation_initializer_->velocity();
        p_antenna_enu = gps_orientation_initializer_->position();
        t_antenna = gps_orientation_initializer_->stamp();
    }

    /*
     * q_tilt contains roll/pitch w.r.t body/base frame.
     * q_yaw contains GPS-derived yaw (if active).
     */
    Eigen::Quaterniond q_enu_base = q_yaw * q_tilt;
    q_enu_base.normalize();

    // Initial position/velocity in ENU frame (if gps active)
    Eigen::Vector3d v_init_enu = Eigen::Vector3d::Zero();
    v_init_enu.head(2) = v_antenna_enu;

    Eigen::Vector3d p_init_enu = Eigen::Vector3d::Zero();
    p_init_enu.head(2) = p_antenna_enu;
   
    const Eigen::Matrix3d R_enu_base = q_enu_base.toRotationMatrix();
    Eigen::Vector3d initial_body_position_enu =
        p_init_enu - R_enu_base * gps_lever_arm_;

    // Set current state time
    if( (t_imu > 0.0) && (t_antenna > 0.0))
    {
        state_.time = (t_imu > t_antenna) ? t_imu : t_antenna;
    }
    else if(t_imu > 0.0)
    {
        state_.time = t_imu;
    }
    else if(t_antenna > 0.0)
    {
        state_.time = t_antenna;
    }
    else{
        state_.time = 0.0;
    }

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
        "Initial RPY: [%.3f, %.3f, %.3f] deg, position: [%.3f, %.3f, %.3f] m, velocity: [%.3f, %.3f, %.3f] m/s (antenna fix: [%.3f, %.3f, %.3f] m @ t_antenna=%.3f s, t_ref=%.3f s)",
        rpy.x(),
        rpy.y(),
        rpy.z(),
        state_.p.x(),
        state_.p.y(),
        state_.p.z(),
        state_.v.x(),
        state_.v.y(),
        state_.v.z(),
        p_init_enu.x(),
        p_init_enu.y(),
        p_init_enu.z(),
        t_antenna,
        state_.time);
}


/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////        CONVERSION HELPERS           /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

double INSEstimator::stampToSec(const rclcpp::Time& header_stamp)
{
    const double t = (use_receive_stamp_) ? this->get_clock()->now().seconds() : header_stamp.seconds();

    if (!filter_time_initialized_) 
        initializeTime(t);

    return t - filter_init_time_;
}

void INSEstimator::initializeTime(double t)
{
    filter_init_time_ = t;

    // Time used internally by the estimator.
    filter_time_ = 0.0;

    filter_time_initialized_ = true;
}

void INSEstimator::handleTimeReset(double t)
{
    RCLCPP_WARN(
        this->get_logger(),
        "Resetting estimator time reference: %.6f -> %.6f",
        filter_init_time_,
        t);
    
    // Start a new time epoch.
    filter_init_time_ = t;

    // The estimator clock starts again from zero.
    filter_time_ = 0.0;

    // Correct time base of backend filter
    iESEKF::group_to_state(filter_.getState(), state_);
    state_.time = filter_time_;
    setState();

    // Empty buffers
    meas_handler_.clear();
}

void INSEstimator::handleTimeJump(double t, double dt)
{
    RCLCPP_WARN(
        this->get_logger(),
        "Large measurement time jump detected: dt = %.6f s. ",
        dt);

    handleTimeReset(t);
}

void INSEstimator::from_ros_to_ins(const sensor_msgs::msg::Imu& in, iESEKF::IMUmeas& out)
{
    out.stamp = stampToSec(in.header.stamp);

    if(last_imu_stamp_ < 0.0){ // first IMU msg received
        out.dt = 0.0;
    }else{
        out.dt = out.stamp - last_imu_stamp_;
    }
    last_imu_stamp_ = out.stamp;

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

    // Time
    out.time = stampToSec(in.header.stamp);
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

    // Time
    out.time = stampToSec(in.header.stamp);
}

void INSEstimator::from_ins_to_ros(const ins_ros::State& in, nav_msgs::msg::Odometry& out,
                                    const std::optional<ROSCovariance>& pose_cov,
                                    const std::optional<ROSCovariance>& twist_cov)
{
    // out.header.stamp = rclcpp::Time(static_cast<int64_t>(in.time * 1e9));
    out.header.stamp = this->get_clock()->now();
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
    if (pose_cov.has_value()) {
        out.pose.covariance = *pose_cov;
    }else{
        auto pose_cov_vec = iESEKF::get_pose_covariance(filter_.getCovariance());
        for (int i = 0; i < 36; ++i)
            out.pose.covariance[i]  = pose_cov_vec[i];
    }

    if(twist_cov.has_value()) {
        out.twist.covariance = *twist_cov;
    }else{
        auto twist_cov_vec = iESEKF::get_velocity_covariance(filter_.getCovariance());
        for (int i = 0; i < 36; ++i)
            out.twist.covariance[i]  = twist_cov_vec[i];
    }

    // add gyro covariance
    out.twist.covariance[21] = gyro_noise_;
    out.twist.covariance[28] = gyro_noise_;
    out.twist.covariance[25] = gyro_noise_;
}

void INSEstimator::from_ins_to_ros(const ins_ros::State& in, geometry_msgs::msg::PoseWithCovarianceStamped& out,
                                    const std::optional<ROSCovariance>& pose_cov)
{
    // out.header.stamp = rclcpp::Time(static_cast<int64_t>(in.time * 1e9));
    out.header.stamp = this->get_clock()->now();
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
    if (pose_cov.has_value()) {
        out.pose.covariance = *pose_cov;
    }else{
        auto pose_cov_vec = iESEKF::get_pose_covariance(filter_.getCovariance());
        for (int i = 0; i < 36; ++i)
            out.pose.covariance[i]  = pose_cov_vec[i];
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

    if (dt > 0.0)
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
            "IMU dt is negative or zero (dt=%.6f). Skipping lever-arm correction.",
            dt);
    }

    // Store values for next measurement.
    previous_omega_base_ = omega_base;

    // Output
    imu.gyro = omega_base;
    imu.accel = accel_base_corrected;

    return true;
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
