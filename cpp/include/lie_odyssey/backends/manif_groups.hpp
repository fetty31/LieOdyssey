#pragma once
#include <manif/SO3.h>
#include <manif/SE3.h>

namespace lie_odyssey {

struct SO3Manif {
    manif::SO3d g;
    static SO3Manif Exp(const Eigen::Vector3d& w) { return { manif::SO3d::Exp(w) }; }
    Eigen::Vector3d Log() const { return g.Log(); }
    SO3Manif operator*(const SO3Manif& o) const { return { g * o.g }; }
    SO3Manif Inverse() const { return { g.inverse() }; }
    Eigen::Matrix3d Adjoint() const { return g.adj(); }
};

struct SE3Manif {
    manif::SE3d g;
    static SE3Manif Exp(const Eigen::Matrix<double,6,1>& xi) { return { manif::SE3d::Exp(xi) }; }
    Eigen::Matrix<double,6,1> Log() const { return g.Log(); }
    SE3Manif operator*(const SE3Manif& o) const { return { g * o.g }; }
    SE3Manif Inverse() const { return { g.inverse() }; }
    Eigen::Matrix<double,6,6> Adjoint() const { return g.adj(); }
};

} // namespace lie_odyssey
