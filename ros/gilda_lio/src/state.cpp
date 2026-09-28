#include "gilda_lio/state.hpp"
#include "gilda_lio/ekf.hpp"

void gilda_lio::State::update(double t) {
    double dt = t - this->time;
    if (dt <= 0.0) return;  

    gilda_lio::iESEKF::Tangent 
        dx = gilda_lio::iESEKF::f_state(*this);

    gilda_lio::iESEKF::Group group;
    gilda_lio::iESEKF::state_to_group(*this, group);

    group.plus(dx * gilda_lio::iESEKF::Scalar(dt));

    gilda_lio::iESEKF::group_to_state(group, *this);
}   

Eigen::Isometry3f gilda_lio::State::get_transform() const {
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    T.translate(p);
    T.rotate(q);
    return T;
}

Eigen::Isometry3f gilda_lio::State::get_inv_transform() const {
    Eigen::Isometry3f T = Eigen::Isometry3f::Identity();
    Eigen::Matrix3f R = q.toRotationMatrix();
    T.translate(-R.transpose()*p);
    T.rotate(R.transpose());
    return T;
}

Eigen::Matrix3f gilda_lio::State::get_R() const {
    return q.toRotationMatrix();
}