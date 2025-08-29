#pragma once
// liepp_groups.hpp
//
// Thin, zero-cost adapters for Lie++ groups to a unified API used by
// lie_odyssey::LieGroup<T>. This file **does not** implement algorithms; it just
// normalizes naming & signatures across backends.
//
// Depends on Lie++ (https://github.com/aau-cns/Lie-plusplus)
// and Eigen3.
//
// Exposes:
//  - lie_odyssey::SO3LiePP<Scalar>
//  - lie_odyssey::SE3LiePP<Scalar>
//  - lie_odyssey::SE23LiePP<Scalar>
//  - lie_odyssey::SEn3LiePP<Scalar, N>
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
// Convenience accessors where available (forwarded to Lie++):
//   - SO3: q() [quaternion], R() [rotation matrix]
//   - SE3: q(), R(), p()
//   - SE23: q(), R(), v(), p()
//
// Notes:
//  - Lie++ (per README examples) uses static `Group::exp(tau)` and
//    static `Group::log(group)`. Adjoint is a **member**: `X.Adjoint()`.
//  - Header paths (groups/SEn.hpp, groups/SO3.hpp) follow the README usage.
//  - If you use only SEn3, you can include <groups/SEn.hpp> alone.

#include <Eigen/Core>
#include <Eigen/Geometry>

// Lie++ includes
// SEn3 (covers SE3 for N=1, SE23 for N=2, ...):
#include <groups/SEn.hpp>
// SO3 (separate base group):
#include <groups/SO3.hpp>
// Gal (separate base group):
#include <groups/Gal.hpp>
// TG (separate base group):
#include <groups/TG.hpp>
// SDB (separate base group):
#include <groups/SDB.hpp>

namespace lie_odyssey {

// ------------------------------- SO(3) ---------------------------------

template <typename _Scalar = double>
struct SO3LiePP {
  using Scalar         = _Scalar;
  using Native         = group::SO3<Scalar>;
  using Tangent        = Eigen::Matrix<Scalar, 3, 1>;
  using MatrixType     = Eigen::Matrix<Scalar, 3, 3>;
  using AdjointMatrix  = Eigen::Matrix<Scalar, 3, 3>;

  Native g;

  SO3LiePP() : g() {}
  explicit SO3LiePP(const Native& gg) : g(gg) {}

  // Exponential / Logarithm (Lie++ style: static Exp/Log)
  static SO3LiePP Exp(const Tangent& w) { return SO3LiePP(Native::exp(w)); }
  static Tangent  Log(const SO3LiePP& X) { return Native::log(X.g); }

  // Group ops
  SO3LiePP operator*(const SO3LiePP& other) const { return SO3LiePP(g * other.g); }
  SO3LiePP Inverse() const { return SO3LiePP(g.inverse()); }

  // Adjoint (member in Lie++)
  AdjointMatrix Adjoint() const { return g.Adjoint(); }

  // Matrix representation
  MatrixType asMatrix() const { return g.asMatrix(); }

  // Convenience accessors (forward to Lie++)
  // Rotation matrix
  MatrixType R() const { return g.asMatrix(); }
  // Quaternion (Eigen::Quaternion<Scalar>)
  auto q() const { return g.q(); }
};

// ------------------------------- SE(3) ---------------------------------
//
// In Lie++: SE(3) is SEn3<Scalar, 1>. We expose a dedicated wrapper for
// ergonomic typedefs and SE(3)-specific accessors.

template <typename _Scalar = double>
struct SE3LiePP {
  using Scalar         = _Scalar;
  using Native         = group::SEn3<Scalar, 1>;
  using Tangent        = Eigen::Matrix<Scalar, 6, 1>;
  using MatrixType     = Eigen::Matrix<Scalar, 4, 4>;
  using AdjointMatrix  = Eigen::Matrix<Scalar, 6, 6>;

  Native g;

  SE3LiePP() : g() {}
  explicit SE3LiePP(const Native& gg) : g(gg) {}
  // Construct from rotation + position if desired
  SE3LiePP(const typename SO3LiePP<Scalar>::Native& R,
           const Eigen::Matrix<Scalar, 3, 1>& p)
      : g(typename SO3LiePP<Scalar>::Native(R), std::array<Eigen::Matrix<Scalar,3,1>,1>{p}) {}

  static SE3LiePP Exp(const Tangent& xi) { return SE3LiePP(Native::exp(xi)); }
  static Tangent   Log(const SE3LiePP& X) { return Native::log(X.g); }

  SE3LiePP operator*(const SE3LiePP& other) const { return SE3LiePP(g * other.g); }
  SE3LiePP Inverse() const { return SE3LiePP(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.Adjoint(); }
  MatrixType    asMatrix() const { return g.asMatrix(); }

  // Convenience accessors (forward to Lie++):
  auto q() const { return g.q(); }                                    // quaternion
  auto R() const { return g.R(); }                                    // rotation matrix (if available; else asMatrix().template block<3,3>(0,0))
  Eigen::Matrix<Scalar,3,1> p() const { return g.p(); }               // position
};

// ------------------------------ SE₂(3) ---------------------------------
//
// In Lie++: SE₂(3) is SEn3<Scalar, 2>. We expose a dedicated wrapper because
// velocity `v()` is available in addition to pose.

template <typename _Scalar = double>
struct SE23LiePP {
  using Scalar         = _Scalar;
  using Native         = group::SEn3<Scalar, 2>;
  using Tangent        = Eigen::Matrix<Scalar, 9, 1>;
  using MatrixType     = Eigen::Matrix<Scalar, 5, 5>;
  using AdjointMatrix  = Eigen::Matrix<Scalar, 9, 9>;

  Native g;

  SE23LiePP() : g() {}
  explicit SE23LiePP(const Native& gg) : g(gg) {}
  SE23LiePP(const typename SO3LiePP<Scalar>::Native& R,
            const Eigen::Matrix<Scalar, 3, 1>& v,
            const Eigen::Matrix<Scalar, 3, 1>& p)
      : g(typename SO3LiePP<Scalar>::Native(R), std::array<Eigen::Matrix<Scalar,3,1>,2>{v, p}) {}

  static SE23LiePP Exp(const Tangent& xi) { return SE23LiePP(Native::exp(xi)); }
  static Tangent    Log(const SE23LiePP& X) { return Native::log(X.g); }

  SE23LiePP operator*(const SE23LiePP& other) const { return SE23LiePP(g * other.g); }
  SE23LiePP Inverse() const { return SE23LiePP(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.Adjoint(); }
  MatrixType    asMatrix() const { return g.asMatrix(); }

  // Accessors present in README example for SE2(3)
  auto q() const { return g.q(); }                                    // quaternion
  auto R() const { return g.R(); }                                    // rotation matrix (if provided)
  Eigen::Matrix<Scalar,3,1> v() const { return g.v(); }               // velocity
  Eigen::Matrix<Scalar,3,1> p() const { return g.p(); }               // position
};

// ------------------------------ SEn(3) ---------------------------------
//
// Generic extended pose with N translational 3-vectors (N >= 1):
//  * N = 1 -> SE(3)
//  * N = 2 -> SE₂(3)
//  * N > 2 -> used in SLAM-style stacks
//
// Accessors beyond the first (e.g., v(), p()) are not standardized here;
// you can reach the native `g` to call the specific methods if needed.

template <typename _Scalar, int N>
struct SEn3LiePP {
  static_assert(N >= 1, "SEn3LiePP requires N >= 1");

  using Scalar         = _Scalar;
  using Native         = group::SEn3<Scalar, N>;
  using Tangent        = Eigen::Matrix<Scalar, 3 * (N + 1), 1>; // 3 for so(3) + 3*N translational
  using MatrixType     = Eigen::Matrix<Scalar, (N + 2), (N + 2)>;
  using AdjointMatrix  = Eigen::Matrix<Scalar, 3 * (N + 1), 3 * (N + 1)>;

  Native g;

  SEn3LiePP() : g() {}
  explicit SEn3LiePP(const Native& gg) : g(gg) {}

  static SEn3LiePP Exp(const Tangent& xi) { return SEn3LiePP(Native::exp(xi)); }
  static Tangent    Log(const SEn3LiePP& X) { return Native::log(X.g); }

  SEn3LiePP operator*(const SEn3LiePP& other) const { return SEn3LiePP(g * other.g); }
  SEn3LiePP Inverse() const { return SEn3LiePP(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.Adjoint(); }
  MatrixType    asMatrix() const { return g.asMatrix(); }

  // Common accessors likely available in Lie++:
  auto q() const { return g.q(); }   // quaternion
  auto R() const { return g.R(); }   // rotation matrix (if implemented)
  // For N >= 1, p() usually exists; for N >= 2, v() usually exists:
  // Expose them conditionally with if constexpr.
  template <int M = N>
  auto p() const -> std::enable_if_t<(M >= 1), Eigen::Matrix<Scalar,3,1>> {
    return g.p();
  }
  template <int M = N>
  auto v() const -> std::enable_if_t<(M >= 2), Eigen::Matrix<Scalar,3,1>> {
    return g.v();
  }
};

// ------------------------------- Gal(3) ---------------------------------

template <typename _Scalar = double>
struct Gal3LiePP {
  using Scalar = _Scalar;
  using Native = group::Gal3<Scalar>;
  using Tangent = Eigen::Matrix<Scalar, 6, 1>;
  using MatrixType = Eigen::Matrix<Scalar, 6, 6>;
  using AdjointMatrix = Eigen::Matrix<Scalar, 6, 6>;

  Native g;

  Gal3LiePP() : g() {}
  explicit Gal3LiePP(const Native& gg) : g(gg) {}

  static Gal3LiePP Exp(const Tangent& xi) { return Gal3LiePP(Native::exp(xi)); }
  static Tangent Log(const Gal3LiePP& X) { return Native::log(X.g); }

  Gal3LiePP operator*(const Gal3LiePP& other) const { return Gal3LiePP(g * other.g); }
  Gal3LiePP Inverse() const { return Gal3LiePP(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.Adjoint(); }
  MatrixType asMatrix() const { return g.asMatrix(); }
};

// ------------------------------- TG ---------------------------------

template <typename _Scalar = double>
struct TGLiePP {
  using Scalar = _Scalar;
  using Native = group::TG<Scalar>;
  using Tangent = Eigen::Matrix<Scalar, 9, 1>;
  using MatrixType = Eigen::Matrix<Scalar, 6, 6>;
  using AdjointMatrix = Eigen::Matrix<Scalar, 9, 9>;

  Native g;

  TGLiePP() : g() {}
  explicit TGLiePP(const Native& gg) : g(gg) {}

  static TGLiePP Exp(const Tangent& xi) { return TGLiePP(Native::exp(xi)); }
  static Tangent Log(const TGLiePP& X) { return Native::log(X.g); }

  TGLiePP operator*(const TGLiePP& other) const { return TGLiePP(g * other.g); }
  TGLiePP Inverse() const { return TGLiePP(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.Adjoint(); }
  MatrixType asMatrix() const { return g.asMatrix(); }
};

// ------------------------------- SDB ---------------------------------

template <typename _Scalar = double>
struct SDBLiePP {
  using Scalar = _Scalar;
  using Native = group::SDB<Scalar>;
  using Tangent = Eigen::Matrix<Scalar, 9, 1>;
  using MatrixType = Eigen::Matrix<Scalar, 6, 6>;
  using AdjointMatrix = Eigen::Matrix<Scalar, 9, 9>;

  Native g;

  SDBLiePP() : g() {}
  explicit SDBLiePP(const Native& gg) : g(gg) {}

  static SDBLiePP Exp(const Tangent& xi) { return SDBLiePP(Native::exp(xi)); }
  static Tangent Log(const SDBLiePP& X) { return Native::log(X.g); }

  SDBLiePP operator*(const SDBLiePP& other) const { return SDBLiePP(g * other.g); }
  SDBLiePP Inverse() const { return SDBLiePP(g.inverse()); }

  AdjointMatrix Adjoint() const { return g.Adjoint(); }
  MatrixType asMatrix() const { return g.asMatrix(); }
};

} // namespace lie_odyssey
