#pragma once
// sophus_groups.hpp
//
// Thin, zero-cost adapters for Sophus groups to a unified API used by
// lie_odyssey::LieGroup<T>. This file **does not** implement algorithms; it just
// normalizes naming & signatures across backends.
//
// Depends on Sophus (https://github.com/strasdat/Sophus)
// and Eigen3.
//
// Exposes:
//  - lie_odyssey::SO3Sophus<Scalar>
//  - lie_odyssey::SE3Sophus<Scalar>
//  - lie_odyssey::SO2Sophus<Scalar>
//  - lie_odyssey::SE2Sophus<Scalar>
//
// Common interface:
//   using Scalar, Tangent, AdjointMatrix, MatrixType, Native;
//   static Wrapper Exp(const Tangent&);
//   static Tangent Log(const Wrapper&);
//   Wrapper operator*(const Wrapper&) const;
//   Wrapper Inverse() const;
//   AdjointMatrix Adjoint() const;
//   MatrixType asMatrix() const;
//
// Convenience accessors:
//   - SO3: matrix(), unit_quaternion()
//   - SE3: matrix(), unit_quaternion(), translation()
//   - SE2: matrix(), translation()
//

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>
#include <sophus/so2.hpp>
#include <sophus/se2.hpp>

namespace lie_odyssey {

// ------------------------------ SO(3) ---------------------------------

template <typename _Scalar = double>
struct SO3Sophus {
  using Scalar        = _Scalar;
  using Native        = Sophus::SO3<Scalar>;
  using Tangent       = Eigen::Matrix<Scalar, 3, 1>;
  using MatrixType    = Eigen::Matrix<Scalar, 3, 3>;
  using AdjointMatrix = Eigen::Matrix<Scalar, 3, 3>;

  Native g;

  SO3Sophus() : g(Native::identity()) {}
  explicit SO3Sophus(const Native& gg) : g(gg) {}

  static SO3Sophus Exp(const Tangent& tau) { return SO3Sophus(Native::exp(tau)); }
  static Tangent   Log(const SO3Sophus& X) { return X.g.log(); }

  SO3Sophus operator*(const SO3Sophus& other) const { return SO3Sophus(g * other.g); }
  SO3Sophus Inverse() const { return SO3Sophus(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.matrix(); } // For SO3, adjoint = rotation
  MatrixType    asMatrix() const { return g.matrix(); }

  MatrixType matrix() const { return g.matrix(); }
  Eigen::Quaternion<Scalar> unit_quaternion() const { return g.unit_quaternion(); }
};

// ------------------------------ SE(3) ---------------------------------

template <typename _Scalar = double>
struct SE3Sophus {
  using Scalar        = _Scalar;
  using Native        = Sophus::SE3<Scalar>;
  using Tangent       = Eigen::Matrix<Scalar, 6, 1>;
  using MatrixType    = Eigen::Matrix<Scalar, 4, 4>;
  using AdjointMatrix = Eigen::Matrix<Scalar, 6, 6>;

  Native g;

  SE3Sophus() : g(Native::identity()) {}
  explicit SE3Sophus(const Native& gg) : g(gg) {}
  SE3Sophus(const Eigen::Quaternion<Scalar>& q, const Eigen::Matrix<Scalar,3,1>& p)
      : g(q, p) {}

  static SE3Sophus Exp(const Tangent& xi) { return SE3Sophus(Native::exp(xi)); }
  static Tangent   Log(const SE3Sophus& X) { return X.g.log(); }

  SE3Sophus operator*(const SE3Sophus& other) const { return SE3Sophus(g * other.g); }
  SE3Sophus Inverse() const { return SE3Sophus(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.Adj(); }
  MatrixType    asMatrix() const { return g.matrix(); }

  Eigen::Quaternion<Scalar> unit_quaternion() const { return g.unit_quaternion(); }
  Eigen::Matrix<Scalar,3,1> translation() const { return g.translation(); }
};

// ------------------------------ SO(2) ---------------------------------

template <typename _Scalar = double>
struct SO2Sophus {
  using Scalar        = _Scalar;
  using Native        = Sophus::SO2<Scalar>;
  using Tangent       = Scalar;
  using MatrixType    = Eigen::Rotation2D<Scalar>;
  using AdjointMatrix = Eigen::Matrix<Scalar, 1, 1>;

  Native g;

  SO2Sophus() : g(Native::identity()) {}
  explicit SO2Sophus(const Native& gg) : g(gg) {}

  static SO2Sophus Exp(const Tangent& theta) { return SO2Sophus(Native::exp(theta)); }
  static Tangent    Log(const SO2Sophus& X) { return X.g.log(); }

  SO2Sophus operator*(const SO2Sophus& other) const { return SO2Sophus(g * other.g); }
  SO2Sophus Inverse() const { return SO2Sophus(g.inverse()); }

  AdjointMatrix Adjoint() const { return AdjointMatrix::Identity(); }
  MatrixType    asMatrix() const { return g.matrix(); }
};

// ------------------------------ SE(2) ---------------------------------

template <typename _Scalar = double>
struct SE2Sophus {
  using Scalar        = _Scalar;
  using Native        = Sophus::SE2<Scalar>;
  using Tangent       = Eigen::Matrix<Scalar, 3, 1>;
  using MatrixType    = Eigen::Matrix<Scalar, 3, 3>;
  using AdjointMatrix = Eigen::Matrix<Scalar, 3, 3>;

  Native g;

  SE2Sophus() : g(Native::identity()) {}
  explicit SE2Sophus(const Native& gg) : g(gg) {}
  SE2Sophus(const Eigen::Rotation2D<Scalar>& R, const Eigen::Matrix<Scalar,2,1>& t)
      : g(R.angle(), t) {}

  static SE2Sophus Exp(const Tangent& xi) { return SE2Sophus(Native::exp(xi)); }
  static Tangent    Log(const SE2Sophus& X) { return X.g.log(); }

  SE2Sophus operator*(const SE2Sophus& other) const { return SE2Sophus(g * other.g); }
  SE2Sophus Inverse() const { return SE2Sophus(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.Adj(); }
  MatrixType    asMatrix() const { return g.matrix(); }

  Eigen::Matrix<Scalar,2,1> translation() const { return g.translation(); }
};

}  // namespace lie_odyssey
