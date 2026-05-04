#include "lio_ros/state.hpp"
#include "lio_ros/ekf.hpp"

void lio_ros::State::update(double t) {
    double dt = t - this->time;
    if (dt <= 0.0) return;  

    lio_ros::iESEKF::Tangent 
        dx = lio_ros::iESEKF::f_state(*this);

    lio_ros::iESEKF::Group group;
    lio_ros::iESEKF::state_to_group(*this, group);

    group.plus(dx * lio_ros::iESEKF::Scalar(dt));

    lio_ros::iESEKF::group_to_state(group, *this);
}   

Eigen::Isometry3f lio_ros::State::get_transform() const {
    Eigen::Isometry3f T;
    T.translate(p);
    T.rotate(q);
    return T;
}

Eigen::Isometry3f lio_ros::State::get_inv_transform() const {
    Eigen::Isometry3f T;
    Eigen::Matrix3f R = q.toRotationMatrix();
    T.translate(-R.transpose()*p);
    T.rotate(R.transpose());
    return T;
}