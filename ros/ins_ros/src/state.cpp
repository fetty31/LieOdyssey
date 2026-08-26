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