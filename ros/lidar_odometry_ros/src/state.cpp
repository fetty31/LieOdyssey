#include "lidar_odometry_ros/objects/state.hpp"

// class lidar_odometry_ros::State
    // public

        lidar_odometry_ros::State::State() : time(0.0) { 
            this->q   = Eigen::Quaternionf::Identity();
            this->p   = Eigen::Vector3f::Zero();
            this->v   = Eigen::Vector3f::Zero();
            this->w   = Eigen::Vector3f::Zero();
            this->a   = Eigen::Vector3f::Zero();
            this->g   = Eigen::Vector3f::Zero();

            this->b.gyro  = Eigen::Vector3f::Zero();
            this->b.accel = Eigen::Vector3f::Zero();
        }


        lidar_odometry_ros::State::State(const iESEKF::Group& group){

            iESEKF::V3 position, velocity, gravity, bg, ba;
            iESEKF::Quat orientation;
            iESEKF::group_to_state(group, position, orientation, velocity, bg, ba, gravity);

            // Odom
            this->q = orientation.cast<float>();
            this->p = position.cast<float>();
            this->v = velocity.cast<float>();

            // Gravity
            this->g = gravity.cast<float>();

            // IMU bias
            this->b.gyro = bg.cast<float>();
            this->b.accel = ba.cast<float>();
        }

        lidar_odometry_ros::State::State(const iESEKF::Group& g, double t) : lidar_odometry_ros::State::State(g) { 
            this->time = t;
        }
        
        lidar_odometry_ros::State::State(const iESEKF::Group& g, double t,
                                Eigen::Vector3f a, Eigen::Vector3f w) : lidar_odometry_ros::State::State(g, t) {
            this->a = a;
            this->w = w;
        }

        lidar_odometry_ros::State::State(Eigen::Matrix4f& T){

            // Get tranform matrix
            Eigen::Matrix3f R = T.block(0, 0, 3, 3);
            this->q = R;

            this->p = T.block(0, 3, 3, 1);
        }

        void lidar_odometry_ros::State::update(double t)
        {
            using Scalar = iESEKF::Scalar;

            double dt = t - this->time;
            if (dt <= 0.0) return;

            // Build group from current state
            iESEKF::Group group;
            iESEKF::state_to_group(this->p.cast<Scalar>(),
                                this->q.cast<Scalar>(),
                                this->v.cast<Scalar>(),
                                this->b.gyro.cast<Scalar>(),
                                this->b.accel.cast<Scalar>(),
                                this->g.cast<Scalar>(),
                                group);

            // Build tangent (IMU input)
            lie_odyssey::IMUmeas imu_input;
            imu_input.accel = this->a.cast<Scalar>();
            imu_input.gyro = this->w.cast<Scalar>();
            imu_input.bias.accel = this->b.accel.cast<Scalar>();
            imu_input.bias.gyro = this->b.gyro.cast<Scalar>();
            imu_input.dt = dt;

            auto xi = iESEKF::f_state(group, imu_input); // get tangent increment from IMU input

            // Propagate on Lie group
            group.plus(xi * dt);

            // Back to state
            iESEKF::V3 position, velocity, gravity, bg, ba;
            iESEKF::Quat orientation;

            iESEKF::group_to_state(group, position, orientation, velocity, bg, ba, gravity);

            this->q = orientation.cast<float>();
            this->p = position.cast<float>();
            this->v = velocity.cast<float>();
            this->g = gravity.cast<float>();
            this->b.gyro = bg.cast<float>();
            this->b.accel = ba.cast<float>();
            this->time = t;
        }

        void lidar_odometry_ros::State::operator+=(const lidar_odometry_ros::State& State){
            this->q *= State.q;
            this->p += State.p;
            this->v += State.v;
            this->w += State.w;

            this->b.gyro = State.b.gyro;
            this->b.accel = State.b.accel;

            this->g = State.g;
        }

        Eigen::Matrix4f lidar_odometry_ros::State::get_RT(){
            // Transformation matrix
            Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
            T.block(0, 0, 3, 3) = this->q.toRotationMatrix();
            T.block(0, 3, 3, 1) = this->p;

            return T;
        }

        Eigen::Matrix4f lidar_odometry_ros::State::get_RT_inv(){
            Eigen::Matrix4f Tinv = Eigen::Matrix4f::Identity();
            Eigen::Matrix3f rot = this->q.toRotationMatrix();
            
            Tinv.block(0, 0, 3, 3) = rot.transpose();
            Tinv.block(0, 3, 3, 1) = -rot.transpose()*this->p;

            return Tinv;
        }