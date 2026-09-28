#pragma once

#include "lio_ros/types.hpp"

namespace lio_ros {

struct State {
    Eigen::Vector3f p{Eigen::Vector3f::Zero()};            // position (world)
    Eigen::Quaternionf q{Eigen::Quaternionf::Identity()};  // orientation (world)
    Eigen::Vector3f v{Eigen::Vector3f::Zero()};            // linear velocity
    Eigen::Vector3f g{Eigen::Vector3f{0.f, 0.f, 9.81f}};   // gravity

    Eigen::Vector3f w{Eigen::Vector3f::Zero()};            // angular vel (baselink frame)
    Eigen::Vector3f a{Eigen::Vector3f::Zero()};            // linear accel (baselink frame)

    double time = 0.0;

    struct IMUbias {
        Eigen::Vector3f w{0.f, 0.f, 0.f};
        Eigen::Vector3f a{0.f, 0.f, 0.f};
    } bias;

    State() = default;

    void update(double t);

    Eigen::Isometry3f get_transform() const;
    Eigen::Isometry3f get_inv_transform() const;
};

} // namespace lio_ros
