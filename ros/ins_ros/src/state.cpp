#include "ins_ros/state.hpp"
#include "ins_ros/ekf.hpp"

void ins_ros::State::progress(double t) {
    if (this->time <= 0.0) {
        this->time = t;
        return;
    }
    double dt = t - this->time;
    if (dt <= 0.0) return;  

    this->time = t;

    ins_ros::iESEKF::Tangent 
        dx = ins_ros::iESEKF::f_state(*this);

    ins_ros::iESEKF::Group group;
    ins_ros::iESEKF::state_to_group(*this, group);

    group.plus(dx * ins_ros::iESEKF::Scalar(dt));

    ins_ros::iESEKF::group_to_state(group, *this);
}   

ins_ros::State::Isometry ins_ros::State::get_transform() const {
    ins_ros::State::Isometry T = ins_ros::State::Isometry::Identity();
    T.translate(p);
    T.rotate(q);
    return T;
}

ins_ros::State::Isometry ins_ros::State::get_inv_transform() const {
    ins_ros::State::Isometry T = ins_ros::State::Isometry::Identity();
    Eigen::Matrix<ins_ros::State::Scalar, 3, 3> R = q.toRotationMatrix();
    T.translate(-R.transpose()*p);
    T.rotate(R.transpose());
    return T;
}

ins_ros::State::V3 ins_ros::State::get_body_velocity() const {
    // Transform world velocity to body frame
    Eigen::Matrix<ins_ros::State::Scalar, 3, 3> R = q.toRotationMatrix();
    return R.transpose() * v;
}

ins_ros::State::V3 ins_ros::State::get_rpy() const
{
    ins_ros::State::V3 euler = q.toRotationMatrix().eulerAngles(2, 1, 0);

    ins_ros::State::Scalar yaw   = euler[0] * 180.0 / M_PI;
    ins_ros::State::Scalar pitch = euler[1] * 180.0 / M_PI;
    ins_ros::State::Scalar roll  = euler[2] * 180.0 / M_PI;

    ins_ros::State::Scalar yaw_alt =
        std::remainder(yaw + 180.0, 360.0);

    ins_ros::State::Scalar pitch_alt =
        std::remainder(180.0 - pitch, 360.0);

    ins_ros::State::Scalar roll_alt =
        std::remainder(roll + 180.0, 360.0);

    const ins_ros::State::Scalar norm_current =
        yaw * yaw +
        pitch * pitch +
        roll * roll;

    const ins_ros::State::Scalar norm_alt =
        yaw_alt * yaw_alt +
        pitch_alt * pitch_alt +
        roll_alt * roll_alt;

    if (norm_alt < norm_current)
    {
        yaw   = yaw_alt;
        pitch = pitch_alt;
        roll  = roll_alt;
    }

    return ins_ros::State::V3(roll, pitch, yaw);
}