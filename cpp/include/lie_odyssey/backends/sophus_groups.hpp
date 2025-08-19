#pragma once
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>

namespace lie_odyssey {

struct SO3Sophus {
    Sophus::SO3d g;
    static SO3Sophus Exp(const Eigen::Vector3d& w) { return { Sophus::SO3d::exp(w) }; }
    Eigen::Vector3d Log() const { return g.log(); }
    SO3Sophus operator*(const SO3Sophus& o) const { return { g * o.g }; }
    SO3Sophus Inverse() const { return { g.inverse() }; }
    Eigen::Matrix3d Adjoint() const { return g.Adj(); }
};

struct SE3Sophus {
    Sophus::SE3d g;
    static SE3Sophus Exp(const Eigen::Matrix<double,6,1>& xi) { return { Sophus::SE3d::exp(xi) }; }
    Eigen::Matrix<double,6,1> Log() const { return g.log(); }
    SE3Sophus operator*(const SE3Sophus& o) const { return { g * o.g }; }
    SE3Sophus Inverse() const { return { g.inverse() }; }
    Eigen::Matrix<double,6,6> Adjoint() const { return g.Adj(); }
};

} // namespace lie_odyssey
