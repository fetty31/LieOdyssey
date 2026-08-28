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
    , tf_buffer_(this->get_clock())
{
}

/* //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////          LIFECYCLE TRANSITIONS        /////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////// */

INSEstimator::CallbackReturn INSEstimator::on_configure(const rclcpp_lifecycle::State&)
{
    RCLCPP_DEBUG(get_logger(), "Configuring...");

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

    // Set buffer capacity
    this->imu_buffer_.set_capacity(2000);
    this->state_buffer_.set_capacity(2000);

    // Initialize state 
    this->state_ = ins_ros::State();
    setState();

    // Reset ENU frame
    enu_converter_ = ENUConverter();

    // TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(tf_buffer_);

    // IMU extrinsics
    R_imu_to_base_ = Eigen::Matrix3d::Identity();
    t_imu_to_base_ = State::V3::Zero();
    previous_omega_base_ = State::V3::Zero();
    imu_extrinsics_initialized_ = false;
    last_imu_stamp_ = -1.0;

    // Orientation initializers
    imu_orientation_initializer_ = std::make_unique<init::IMUOrientationInitializer>();
    gps_orientation_initializer_ = std::make_unique<init::GPSOrientationInitializer>();
    orientation_initialized_ = false;

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
    pose_sub_.reset();
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
    RCLCPP_DEBUG(get_logger(), "Error occurred - cleaning up...");

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
    publish_tf_  = get_parameter("tf.publish").as_bool();

    // Filter
    // filter_type_ = get_parameter("filter.type").as_string();
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

        // GPS
    bool gps_enabled = get_parameter("sensors.gps.enabled").as_bool();
    trust_gps_covariance_ = get_parameter("sensors.gps.trust_covariance").as_bool();
    if (!gps_enabled)
    {
        RCLCPP_WARN(get_logger(), "GPS is disabled. The estimator will run without GPS.");
        using_gps_enu_ = false;
    }else{
        gps_topic_ = get_parameter("sensors.gps.topic").as_string();
        double gps_noise_x = get_parameter("sensors.gps.covariance.position.x").as_double();
        double gps_noise_y = get_parameter("sensors.gps.covariance.position.y").as_double();
        double gps_noise_z = get_parameter("sensors.gps.covariance.position.z").as_double();
        gps_noise_ = State::V3(gps_noise_x, gps_noise_y, gps_noise_z);
        double lever_arm_x = get_parameter("sensors.gps.lever_arm.x").as_double();
        double lever_arm_y = get_parameter("sensors.gps.lever_arm.y").as_double();
        double lever_arm_z = get_parameter("sensors.gps.lever_arm.z").as_double();
        gps_lever_arm_ = State::V3(lever_arm_x, lever_arm_y, lever_arm_z);
    }
    
        // Wheel odometry
    bool wheel_odom_enabled = get_parameter("sensors.wheel_odom.enabled").as_bool();
    if (wheel_odom_enabled)
    {
        wheel_odom_topic_ = get_parameter("sensors.wheel_odom.topic").as_string();
        double wheel_odom_noise_x = get_parameter("sensors.wheel_odom.covariance.velocity.x").as_double();
        double wheel_odom_noise_y = get_parameter("sensors.wheel_odom.covariance.velocity.y").as_double();
        double wheel_odom_noise_z = get_parameter("sensors.wheel_odom.covariance.velocity.z").as_double();
        wheel_odom_noise_ = State::V3(wheel_odom_noise_x, wheel_odom_noise_y, wheel_odom_noise_z);
    }
    
        // 3D Pose
    bool pose_enabled = get_parameter("sensors.pose.enabled").as_bool();
    if (pose_enabled)
    {
        pose_topic_ = get_parameter("sensors.pose.topic").as_string();
    }
    
        // Magnetometer
    bool mag_enabled = get_parameter("sensors.mag.enabled").as_bool();
    if (mag_enabled)
    {
        mag_topic_ = get_parameter("sensors.mag.topic").as_string();
    }

        // Barometer
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
        using_gps_enu_ = true;
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

    if(!pose_topic_.empty())
    {
        pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic_, 
            sensor_qos,
            std::bind(&INSEstimator::pose_callback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "  Pose:       %s", pose_topic_.c_str());
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
    state_pub_ = create_publisher<nav_msgs::msg::Odometry>("/ins_ros/odom", 1);
    pose_pub_  = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/ins_ros/pose", 1);
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

    // Compute dt from last IMU stamp
    if (last_imu_stamp_ < 0.0)
    {
        last_imu_stamp_ = imu.stamp;
        previous_omega_base_ = imu.gyro;
        return;
    }
    imu.dt = imu.stamp - last_imu_stamp_;
    last_imu_stamp_ = imu.stamp;

    // Transform IMU measurements into base_link frame if necessary.
    if (!transform_imu_to_base_link(msg, imu))
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Skipping IMU propagation.");
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
            state_.bias.a = imu_orientation_initializer_->accelerometer_bias().cast<iESEKF::Scalar>();
            state_.bias.w = imu_orientation_initializer_->gyroscope_bias().cast<iESEKF::Scalar>();
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
    if (using_gps_enu_)
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

        if ( (!gps_orientation_initializer_->initialized()) && using_gps_enu_ )
            return;

        initialize_orientation();
    }

    // Filter prediction
    filter_.predict(imu);

    // Update local state
    iESEKF::group_to_state(filter_.getState(), state_);
    state_.w = imu.gyro;
    state_.a = imu.accel;
    state_.time = imu.stamp;

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

    iESEKF::gps::GPSMeasurement meas;
    meas.position_enu = p_gps_enu.cast<iESEKF::Scalar>();
    meas.position_enu(2) = 0.0; // ignore altitude for now
    meas.lever_arm = gps_lever_arm_.cast<iESEKF::Scalar>();

    RCLCPP_DEBUG(
        get_logger(),
        "GPS measurement in ENU frame: [%.3f, %.3f, %.3f] m",
        meas.position_enu.x(),
        meas.position_enu.y(),
        meas.position_enu.z());

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
    
    // DEBUG 
    const auto rpy = state_.get_rpy();
    const auto v_body = state_.get_body_velocity();

    RCLCPP_DEBUG(
        get_logger(),
        "\n"
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
        state_.time,
        state_.p.x(), state_.p.y(), state_.p.z(),
        state_.v.x(), state_.v.y(), state_.v.z(),
        v_body.x(), v_body.y(), v_body.z(),
        rpy.x(),
        rpy.y(),
        rpy.z(),
        state_.q.w(), state_.q.x(), state_.q.y(), state_.q.z(),
        state_.g.x(), state_.g.y(), state_.g.z(),
        state_.w.x(), state_.w.y(), state_.w.z(),
        state_.a.x(), state_.a.y(), state_.a.z(),
        state_.bias.w.x(), state_.bias.w.y(), state_.bias.w.z(),
        state_.bias.a.x(), state_.bias.a.y(), state_.bias.a.z());
}

void INSEstimator::wheel_odom_callback(const geometry_msgs::msg::TwistStamped& msg)
{
    iESEKF::Measurement meas = iESEKF::Measurement::Zero(3);
    meas(0) = msg.twist.linear.x;
    meas(1) = msg.twist.linear.y;
    // meas(2) = msg.twist.linear.z;

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

bool INSEstimator::initialize_imu_extrinsics(
    const std::string& imu_frame)
{
    if (imu_frame.empty() || imu_frame == body_frame_)
    {
        RCLCPP_WARN(
            get_logger(),
            "IMU message has an empty frame_id. Assuming %s frame.", 
            body_frame_.c_str());

        R_imu_to_base_.setIdentity();
        t_imu_to_base_.setZero();

        imu_extrinsics_initialized_ = true;
        return true;
    }

    try
    {
        const auto tf = tf_buffer_.lookupTransform(
            body_frame_,
            imu_frame,
            tf2::TimePointZero,
            tf2::durationFromSec(1.0));

        const auto & q = tf.transform.rotation;

        Eigen::Quaternion<State::Scalar> quat(
            q.w, q.x, q.y, q.z);
        quat.normalize();

        R_imu_to_base_ = quat.toRotationMatrix();

        const auto & t = tf.transform.translation;

        const State::V3 t_imu_in_base(
            t.x,
            t.y,
            t.z);

        // Vector from IMU origin to base_link origin,
        // expressed in base_link.
        t_imu_to_base_ = -t_imu_in_base;

        imu_extrinsics_initialized_ = true;

        RCLCPP_INFO(
            get_logger(),
            "Initialized IMU extrinsics: %s -> %s",
            imu_frame.c_str(),
            body_frame_.c_str());

        return true;
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_ERROR(
            get_logger(),
            "Failed to initialize IMU extrinsics from '%s' to '%s': %s",
            imu_frame.c_str(),
            body_frame_.c_str(),
            ex.what());

        return false;
    }
}

bool INSEstimator::transform_imu_to_base_link(
    const sensor_msgs::msg::Imu & msg,
    iESEKF::IMUmeas& imu)
{
    if (!imu_extrinsics_initialized_)
    {
        if (!initialize_imu_extrinsics(msg.header.frame_id))
            return false;
    }

    // Angular velocity
    const State::V3 omega_base =
        R_imu_to_base_ * imu.gyro;

    // Linear acceleration
    const State::V3 accel_base =
        R_imu_to_base_ * imu.accel;
    State::V3 accel_base_corrected = accel_base;

    // Need angular acceleration for the lever-arm correction.
    const double dt = imu.dt;

    if (dt > 0.0 && dt < 0.1)
    {
        const State::V3 alpha_base =
            (omega_base - previous_omega_base_) / dt;

        // a_B = a_I + alpha x r + omega x (omega x r)
        accel_base_corrected +=
            alpha_base.cross(t_imu_to_base_);

        accel_base_corrected +=
            omega_base.cross(
                omega_base.cross(t_imu_to_base_));
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

    Eigen::Quaterniond q_tilt = imu_orientation_initializer_->orientation();
    
    Eigen::Quaterniond q_yaw = Eigen::Quaterniond::Identity();
    if(using_gps_enu_)
        q_yaw = gps_orientation_initializer_->orientation();

    /*
     * q_tilt contains roll/pitch.
     * q_yaw contains GPS-derived yaw (if active).
     */
    Eigen::Quaterniond q_enu_imu =
        q_yaw * q_tilt;

    q_enu_imu.normalize();

    state_.q =
        q_enu_imu.cast<iESEKF::Scalar>();
    
    setState();
    
    // Update filter with initial orientation measurement
    iESEKF::Group group_meas;
    iESEKF::state_to_group(state_, group_meas);
    Eigen::MatrixXd R_pose = Eigen::MatrixXd::Identity(6, 6) * 0.1; // 10cm/0.1rad covariance
    Eigen::MatrixXd R_pose_inv = R_pose.inverse();
    filter_.update<iESEKF::Group, iESEKF::Measurement, iESEKF::HMat>(
        group_meas,
        R_pose, R_pose_inv,
        ins_ros::iESEKF::pose::H_fun);

    orientation_initialized_ = true;

    if(using_gps_enu_)
    {
        RCLCPP_INFO(
            get_logger(),
            "INS orientation initialized taking into account IMU + GPS.");
    }else{
        RCLCPP_INFO(
            get_logger(),
            "INS orientation initialized taking into account IMU (no GPS available).");
    }

    const auto rpy = state_.get_rpy();

    RCLCPP_INFO(
        get_logger(),
        "Initial RPY: [%.3f, %.3f, %.3f] deg",
        rpy.x(),
        rpy.y(),
        rpy.z());
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

