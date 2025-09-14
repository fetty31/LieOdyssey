#ifndef __LIEODYSSEY_BACKENDS_LIEPLUSPLUS_HPP__
#define __LIEODYSSEY_BACKENDS_LIEPLUSPLUS_HPP__
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
//  - lie_odyssey::Gal3LiePP<Scalar>
//  - lie_odyssey::Gal3TGLiePP<Scalar>
//
// Common interface provided by all wrappers:
//   using Native, Tangent, MatrixType, Jacobian;
//   static Wrapper Exp(const Tangent&);
//   static Tangent Log(const Wrapper&);
//   Wrapper operator*(const Wrapper&) const;
//   Wrapper Inverse() const;
//   Native native();
//   Jacobian Adjoint() const;
//   Jacobian invAdjoint() const;
//   Jacobian adjoint(const Tangent&) const;
//   MatrixType asMatrix() const;
//   Jacobian leftJacobian(const Tangent&) const;
//   Jacobian invLeftJacobian(const Tangent&) const;
//   Jacobian rightJacobian(const Tangent&) const;
//   Jacobian invRightJacobian(const Tangent&) const;
//
// Convenience accessors where available (forwarded to Lie++):
//   - SO3: q() [quaternion], R() [rotation matrix]
//   - SE3: q(), R(), p()
//   - SE23: q(), R(), v(), p()
//   - Gal3: q(), R(), v(), p(), t()
//   - Gal3TG: G() [lie group], g() [lie algebra]
//
// Notes:
//  - Lie++ (per README examples) uses static `Group::exp(tau)` and
//    static `Group::log(group)`. Adjoint is a **member**: `X.Adjoint()`.

#include <Eigen/Core>
#include <Eigen/Geometry>

// Lie++ includes
// SEn3 (covers SE3 for N=1, SE23 for N=2, ...):
#include <groups/SEn3.hpp>
// SO3 (separate base group):
#include <groups/SO3.hpp>
// Gal (separate base group):
#include <groups/Gal3.hpp>
// TG (separate base group):
#include <groups/TG.hpp>
// SDB (separate base group):
#include <groups/SDB.hpp>

namespace lie_odyssey {

// ------------------------------- Base ---------------------------------

// Template parameter: Group = the lie++ group type (e.g. group::SO3,
// group::SE3, group::SE23, group::Gal3, group::Gal3TG or other group::Group types).
template<typename Derived, typename Group>
class BaseLiePP {

  public:
    using Native         = Group;
    using Tangent        = typename Native::VectorType;    // Lie Algebra Tangent space 
    using MatrixType     = typename Native::MatrixType;    // Lie Group matrix type
    using Jacobian       = typename Native::TMatrixType;   // Transformation matrix type

    Native g_;

    BaseLiePP() : g_() {}
    explicit BaseLiePP(const Native& gg) : g_(gg) {}

    Native native() { return g_; }

    // Identity utils
    void setIdentity() { this->g_ = Native(); }
    Derived Identity() 
    { 
      return Derived();
    }

    // Exponential / Logarithm (Lie++ style: static Exp/Log)
    static Derived Exp(const Tangent& u) { return Derived(Native::exp(u)); }
    Tangent Log() { return Native::log(g_); }

    // Right Plus/Minus operators
    void plus(Tangent& u){ g_ *= Native::exp(u); } // right plus X' = X ⊕ u
    Tangent minus(Derived& X)
    { 
      const Derived& self = static_cast<const Derived&>(*this);
      return Log( X.Inverse().native()*self.g_ );  // right minus t = Y ⊖ X
    }

    // Group ops
    Derived operator*(const Derived& other) const { return Derived(g_ * other.g_); }
    Derived Inverse() const { return Derived(g_.inv()); }

    // Adjoint 
    Jacobian Adjoint() const { return g_.Adjoint(); }
    Jacobian invAdjoint() const { return g_.invAdjoint(); }
    Jacobian adjoint(const Tangent& u) const { return Native::adjoint(u); }

    // Matrix representation
    MatrixType asMatrix() const { return g_.asMatrix(); }

    // Jacobians (w.r.t perturbation)
    Jacobian leftJacobian(const Tangent& u) const { return Native::leftJacobian(u); }
    Jacobian invLeftJacobian(const Tangent& u) const { return Native::invLeftJacobian(u); }
    Jacobian rightJacobian(const Tangent& u) const { return Native::rightJacobian(u); }
    Jacobian invRightJacobian(const Tangent& u) const { return Native::invRightJacobian(u); }

};

// ------------------------------- SO(3) ---------------------------------

template <typename Scalar = double>
class SO3LiePP : public BaseLiePP<SO3LiePP<Scalar>, group::SO3<Scalar>> {
  
  using Base = BaseLiePP<SO3LiePP<Scalar>, group::SO3<Scalar>>;

  public:
    static constexpr int DoF = 3;

    SO3LiePP() : Base() { }
    explicit SO3LiePP(const typename Base::Native& gg) : Base(gg) { }

  // Convenience accessors (forward to Lie++)
  auto R() const { return this->g_.asMatrix(); }
  auto q() const { return this->g_.q(); }
};

// ------------------------------- SE(3) ---------------------------------
//
// In Lie++: SE(3) is SEn3<Scalar, 1>. We expose a dedicated wrapper for
// ergonomic typedefs and SE(3)-specific accessors.

template <typename Scalar = double>
class SE3LiePP : public BaseLiePP<SE3LiePP<Scalar>, group::SEn3<Scalar, 1>> {
  
  using Base = BaseLiePP<SE3LiePP<Scalar>, group::SEn3<Scalar, 1>>;

  public:
    static constexpr int DoF = 6;

    SE3LiePP() : Base() { }
    explicit SE3LiePP(const typename Base::Native& gg) : Base(gg) { }

    // Convenience accessors (forward to Lie++):
    auto q() const { return this->g_.q(); }               // quaternion
    auto R() const { return this->g_.R(); }               // rotation matrix
    auto p() const { return this->g_.p(); }               // position
};


// ------------------------------ SE₂(3) ---------------------------------
//
// In Lie++: SE₂(3) is SEn3<Scalar, 2>. We expose a dedicated wrapper because
// velocity `v()` is available in addition to pose.

template <typename Scalar = double>
class SE23LiePP : public BaseLiePP<SE23LiePP<Scalar>, group::SEn3<Scalar, 2>> {
  
  using Base = BaseLiePP<SE23LiePP<Scalar>, group::SEn3<Scalar, 2>>;

  public:
    static constexpr int DoF = 9;

    SE23LiePP() : Base() { }
    explicit SE23LiePP(const typename Base::Native& gg) : Base(gg) { }

    // Convenience accessors (forward to Lie++):
    auto q() const { return this->g_.q(); }   // quaternion
    auto R() const { return this->g_.R(); }   // rotation matrix 
    auto v() const { return this->g_.v(); }   // velocity
    auto p() const { return this->g_.p(); }   // position
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

template <typename Scalar, int N>
class SEn3LiePP : public BaseLiePP<SEn3LiePP<Scalar, N>, group::SEn3<Scalar, N>> {
  
  using Base = BaseLiePP<SEn3LiePP<Scalar, N>, group::SEn3<Scalar, N>>;

  public:

    SEn3LiePP() : Base() { }
    explicit SEn3LiePP(const typename Base::Native& gg) : Base(gg) { }

    // Convenience accessors (forward to Lie++):
    auto q() const { return this->g_.q(); }   // quaternion
    auto R() const { return this->g_.R(); }   // rotation matrix
    // For N >= 1, p() exists; for N >= 2, v() exists:
    // Expose them conditionally with if constexpr.
    template <int M = N>
    auto p() const -> std::enable_if_t<(M >= 1), Eigen::Matrix<Scalar,3,1>> {
      return this->g_.p();
    }
    template <int M = N>
    auto v() const -> std::enable_if_t<(M >= 2), Eigen::Matrix<Scalar,3,1>> {
      return this->g_.v();
    }
};

// ------------------------------- Gal(3) ---------------------------------

template <typename Scalar = double>
class Gal3LiePP : public BaseLiePP<Gal3LiePP<Scalar>, group::Gal3<Scalar>> {
  
  using Base = BaseLiePP<Gal3LiePP<Scalar>, group::Gal3<Scalar>>;

  public:
    static constexpr int DoF = 10;

    Gal3LiePP() : Base() { }
    explicit Gal3LiePP(const typename Base::Native& gg) : Base(gg) { }

    // Convenience accessors (forward to Lie++):
    auto q() const { return this->g_.q(); }   // quaternion
    auto R() const { return this->g_.R(); }   // rotation matrix 
    auto v() const { return this->g_.v(); }   // velocity
    auto p() const { return this->g_.p(); }   // position
    auto t() const { return this->g_.s(); }   // scalar (time)
};

// ------------------------------- TG ---------------------------------

template <typename Scalar = double>
class Gal3TGLiePP : public BaseLiePP<Gal3TGLiePP<Scalar>, group::Gal3TG<Scalar>> {
  
  using Base = BaseLiePP<Gal3TGLiePP<Scalar>, group::Gal3TG<Scalar>>;

  public:
    static constexpr int DoF = 16;

    Gal3TGLiePP() : Base() { }
    explicit Gal3TGLiePP(const typename Base::Native& gg) : Base(gg) { }

    // Convenience accessors (forward to Lie++):
    auto G() const { return Gal3LiePP<Scalar>(this->g_.G()); }   // get Lie Group element
    auto g() const { return this->g_.g(); }                      // get Lie Algebra element
   
};

} // namespace lie_odyssey

#endif