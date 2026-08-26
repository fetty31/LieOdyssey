#pragma once

#include <Eigen/Dense>
#include <mutex>

namespace ins_ros {

struct State {

    using Scalar = double;
    using V3 = Eigen::Matrix<Scalar, 3, 1>;
    using Quat = Eigen::Quaternion<Scalar>;
    using Isometry = Eigen::Transform<Scalar, 3, Eigen::Isometry>;

    V3 p{V3::Zero()};            // position (world)
    Quat q{Quat::Identity()};    // orientation (world)
    V3 v{V3::Zero()};            // linear velocity
    V3 g{V3{0., 0., 9.81}};      // gravity

    V3 w{V3::Zero()};            // angular vel (baselink frame)
    V3 a{V3::Zero()};            // linear accel (baselink frame)

    double time = 0.0;

    struct IMUbias {
        V3 w{V3::Zero()};
        V3 a{V3::Zero()};
    } bias;

    State() = default;

    void progress(double t);

    Isometry get_transform() const;
    Isometry get_inv_transform() const;
    V3 get_body_velocity() const;
};

} // namespace ins_ros
