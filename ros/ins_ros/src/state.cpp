#include "ins_ros/state.hpp"
#include "ins_ros/ekf.hpp"

void ins_ros::State::update(double t) {
    double dt = t - this->time;
    if (dt <= 0.0) return;  

    ins_ros::iESEKF::Tangent 
        dx = ins_ros::iESEKF::f_state(*this);

    ins_ros::iESEKF::Group group;
    ins_ros::iESEKF::state_to_group(*this, group);

    group.plus(dx * ins_ros::iESEKF::Scalar(dt));

    ins_ros::iESEKF::group_to_state(group, *this);
}   

Eigen::Isometry3f ins_ros::State::get_transform() const {
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translate(p);
    T.rotate(q);
    return T;
}

Eigen::Isometry3f ins_ros::State::get_inv_transform() const {
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    Eigen::Matrix3f R = q.toRotationMatrix();
    T.translate(-R.transpose()*p);
    T.rotate(R.transpose());
    return T;
}