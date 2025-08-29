#pragma once
// manif_groups.hpp
//
// Thin, zero-cost adapters for Manif groups to a unified API used by
// lie_odyssey::LieGroup<T>. This file **does not** implement algorithms; it just
// normalizes naming & signatures across backends.
//
// Depends on Manif (https://github.com/artivis/manif)
// and Eigen3.
//
// Exposes:
//  - lie_odyssey::SO3Manif<Scalar>
//  - lie_odyssey::SE3Manif<Scalar>
//  - lie_odyssey::SE2Manif<Scalar>
//  - lie_odyssey::RnManif<Scalar,N>
//  - lie_odyssey::SGal3Manif<Scalar> (Special Galilean group, if supported)
//
// Common interface provided by all wrappers:
//   using Scalar, Tangent, AdjointMatrix, MatrixType, Native;
//   static Wrapper Exp(const Tangent&);
//   static Tangent Log(const Wrapper&);
//   Wrapper operator*(const Wrapper&) const;
//   Wrapper Inverse() const;
//   AdjointMatrix Adjoint() const;
//   MatrixType asMatrix() const;
//
// Convenience accessors where available (forwarded to Manif):
//   - SO3: q() [quaternion], R() [rotation matrix]
//   - SE3: q(), R(), p()
//   - SE2: R(), t()
//   - SGal: R(), v(), p()

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <manif/manif.h>

namespace lie_odyssey {

// ------------------------------- SO(3) ---------------------------------

template <typename _Scalar = double>
struct SO3Manif {
  using Scalar        = _Scalar;
  using Native        = manif::SO3<Scalar>;
  using Tangent       = manif::SO3Tangent<Scalar>;
  using MatrixType    = Eigen::Matrix<Scalar,3,3>;
  using AdjointMatrix = Eigen::Matrix<Scalar,3,3>;

  Native g;

  SO3Manif() : g(Native::Identity()) {}
  explicit SO3Manif(const Native& gg) : g(gg) {}

  static SO3Manif Exp(const Tangent& tau) { return SO3Manif(Native::exp(tau)); }
  static Tangent  Log(const SO3Manif& X) { return X.g.log(); }

  SO3Manif operator*(const SO3Manif& other) const { return SO3Manif(g * other.g); }
  SO3Manif Inverse() const { return SO3Manif(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.adj(); }
  MatrixType asMatrix() const { return g.rotation(); }

  // Convenience
  MatrixType R() const { return g.rotation(); }
  Eigen::Quaternion<Scalar> q() const { return g.quat(); }
};

// ------------------------------- SE(3) ---------------------------------

template <typename _Scalar = double>
struct SE3Manif {
  using Scalar        = _Scalar;
  using Native        = manif::SE3<Scalar>;
  using Tangent       = manif::SE3Tangent<Scalar>;
  using MatrixType    = Eigen::Matrix<Scalar,4,4>;
  using AdjointMatrix = Eigen::Matrix<Scalar,6,6>;

  Native g;

  SE3Manif() : g(Native::Identity()) {}
  explicit SE3Manif(const Native& gg) : g(gg) {}
  SE3Manif(const Eigen::Quaternion<Scalar>& q, const Eigen::Matrix<Scalar,3,1>& p)
      : g(q, p) {}

  static SE3Manif Exp(const Tangent& xi) { return SE3Manif(Native::exp(xi)); }
  static Tangent  Log(const SE3Manif& X) { return X.g.log(); }

  SE3Manif operator*(const SE3Manif& other) const { return SE3Manif(g * other.g); }
  SE3Manif Inverse() const { return SE3Manif(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.adj(); }
  MatrixType    asMatrix() const { return g.transform(); }

  // Convenience
  Eigen::Quaternion<Scalar> q() const { return g.quat(); }
  Eigen::Matrix<Scalar,3,3> R() const { return g.rotation(); }
  Eigen::Matrix<Scalar,3,1> p() const { return g.translation(); }
};

// ------------------------------- SE(2) ---------------------------------

template <typename _Scalar = double>
struct SE2Manif {
  using Scalar        = _Scalar;
  using Native        = manif::SE2<Scalar>;
  using Tangent       = manif::SE2Tangent<Scalar>;
  using MatrixType    = Eigen::Matrix<Scalar,3,3>;
  using AdjointMatrix = Eigen::Matrix<Scalar,3,3>;

  Native g;

  SE2Manif() : g(Native::Identity()) {}
  explicit SE2Manif(const Native& gg) : g(gg) {}
  SE2Manif(const Eigen::Rotation2D<Scalar>& R, const Eigen::Matrix<Scalar,2,1>& t)
      : g(R, t) {}

  static SE2Manif Exp(const Tangent& xi) { return SE2Manif(Native::exp(xi)); }
  static Tangent  Log(const SE2Manif& X) { return X.g.log(); }

  SE2Manif operator*(const SE2Manif& other) const { return SE2Manif(g * other.g); }
  SE2Manif Inverse() const { return SE2Manif(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.adj(); }
  MatrixType    asMatrix() const { return g.transform(); }

  // Convenience
  Eigen::Matrix<Scalar,2,2> R() const { return g.rotation(); }
  Eigen::Matrix<Scalar,2,1> t() const { return g.translation(); }
};

// ------------------------------- R^n ---------------------------------
// Manif also defines Euclidean R^n (as Rn) which behaves like a Lie group.

template <typename _Scalar, int N>
struct RnManif {
  using Scalar        = _Scalar;
  using Native        = manif::Rn<Scalar,N>;
  using Tangent       = manif::RnTangent<Scalar,N>;
  using MatrixType    = Eigen::Matrix<Scalar,N,1>;
  using AdjointMatrix = Eigen::Matrix<Scalar,N,N>; // Identity always

  Native g;

  RnManif() : g(Native::Identity()) {}
  explicit RnManif(const Native& gg) : g(gg) {}
  explicit RnManif(const Eigen::Matrix<Scalar,N,1>& v) : g(v) {}

  static RnManif Exp(const Tangent& v) { return RnManif(Native::exp(v)); }
  static Tangent Log(const RnManif& X) { return X.g.log(); }

  RnManif operator+(const RnManif& other) const { return RnManif(g + other.g); }
  RnManif operator-(const RnManif& other) const { return RnManif(g - other.g); }

  // Group identity (translation group)
  RnManif operator*(const RnManif& other) const { return RnManif(g + other.g); }
  RnManif Inverse() const { return RnManif(-g); }

  AdjointMatrix Adjoint() const { return AdjointMatrix::Identity(); }
  MatrixType asMatrix() const { return g.vector(); }

  Eigen::Matrix<Scalar,N,1> v() const { return g.vector(); }
};

// ------------------------------- SGal(3) ---------------------------------
//
// Special Galilean group (rotation, translation, velocity, and time).
// Provided directly by Manif as manif::SGal3<Scalar> with matching Tangent type.

template <typename _Scalar = double>
struct SGal3Manif {
  using Scalar         = _Scalar;
  using Native         = manif::SGal3<Scalar>;
  using Tangent        = manif::SGal3Tangent<Scalar>;
  using MatrixType     = typename Native::Transformation;  // typically 5x5
  using AdjointMatrix  = Eigen::Matrix<Scalar,
                            manif::SGal3<Scalar>::Dim,
                            manif::SGal3<Scalar>::Dim>;

  Native g;

  SGal3Manif() : g(Native::Identity()) {}
  explicit SGal3Manif(const Native& gg) : g(gg) {}

  static SGal3Manif Exp(const Tangent& xi) { return SGal3Manif(Native::Exp(xi)); }
  static Tangent  Log(const SGal3Manif& X) { return Native::Log(X.g); }

  SGal3Manif operator*(const SGal3Manif& other) const { return SGal3Manif(g * other.g); }
  SGal3Manif Inverse() const { return SGal3Manif(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.adj(); }
  MatrixType    asMatrix() const { return g.transform(); }

  // Convenience accessors:
  Eigen::Matrix<Scalar,3,3> R() const { return g.rotation(); }
  Eigen::Matrix<Scalar,3,1> v() const { return g.velocity(); }
  Eigen::Matrix<Scalar,3,1> p() const { return g.translation(); }
  Scalar time() const { return g.time(); }
};

} // namespace lie_odyssey
