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

        void lidar_odometry_ros::State::update(double t){

                // R ⊞ (w - bw - nw)*dt
                // v ⊞ (R*(a - ba - na) + g)*dt
                // p ⊞ (v*dt + 1/2*(R*(a - ba - na) + g)*dt*dt)

                // Time between IMU samples
                double dt = t - this->time;
                if(dt < 0.0) return; // invalid time, do not update

                // Exp orientation
                Eigen::Vector3f w = this->w - this->b.gyro;
                float w_norm = w.norm();
                Eigen::Matrix3f R = Eigen::Matrix3f::Identity();

                if (w_norm > 1.e-7){
                    Eigen::Vector3f r = w / w_norm;
                    Eigen::Matrix3f K;

                    K << 0.0, -r[2], r[1],
                         r[2], 0.0, -r[0],
                         -r[1], r[0], 0.0;

                    float r_ang = w_norm * dt;

                    /// Rodrigues Transformation
                    R += std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
                }

                // Acceleration
                Eigen::Vector3f a0 = this->q._transformVector(this->a - this->b.accel);
                a0 += this->g;

                // Orientation
                Eigen::Quaternionf q_update(R);
                this->q *= q_update;

                // Position
                this->p += this->v*dt + 0.5*a0*dt*dt;

                // Velocity
                this->v += a0*dt;

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