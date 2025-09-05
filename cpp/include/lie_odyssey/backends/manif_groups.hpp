#ifndef __LIEODYSSEY_MANIF_HPP__
#define __LIEODYSSEY_MANIF_HPP__
// manif_groups.hpp
//
// Thin, zero-cost adapters for manif (https://github.com/artivis/manif)
// to a unified API used by lie_odyssey::LieGroup<T>. This file **does not**
// implement algorithms; it just normalizes naming & signatures across backends.
//
// Depends on manif (https://github.com/artivis/manif)
// and Eigen3.
//
// Exposes:
//  - lie_odyssey::SO3Manif<Scalar>
//  - lie_odyssey::SE3<Scalar>
//  - lie_odyssey::SE23Manif<Scalar>
//  - lie_odyssey::Gal3Manif<Scalar>
//
// Common interface provided by all wrappers:
//   using Native, Tangent, MatrixType, TMatrixType;
//   static Wrapper Exp(const Tangent&);
//   static Tangent Log(const Wrapper&);
//   Wrapper operator*(const Wrapper&) const;
//   Wrapper Inverse() const;
//   Native native();
//   TMatrixType Adjoint() const;
//   TMatrixType invAdjoint() const;
//   TMatrixType adjoint(const Tangent&) const;
//   MatrixType asMatrix() const;
//   TMatrixType leftJacobian(const Tangent&) const;
//   TMatrixType invLeftJacobian(const Tangent&) const;
//   TMatrixType rightJacobian(const Tangent&) const;
//   TMatrixType invRightJacobian(const Tangent&) const;
//
// Convenience accessors where available (forwarded to Manif):
//   - SO3: q() [quaternion], R() [rotation matrix]
//   - SE3: q(), R(), p()
//   - SE23: q(), R(), v(), p()
//   - Gal3: q(), R(), v(), p(), t()
//
//
// Relies on the public, documented manif API (see the repository README / docs).
// The manif README documents the following (relevant) member names:
//   - Inverse:          X.inverse()
//   - Composition:      X * Y  (operator*), X.compose(Y)
//   - Exponential:      w.exp()     (where w is a tangent / local vector object)
//   - Logarithm:        X.log()     (returns tangent object / local vector)
//   - Adjoint (manifold): X.adj()
//   - Tangent adjoint:  w.smallAdj()
//   - Plus / Minus:     X + w  (right plus), X.minus(Y), X.plus(w), X.rplus(w)
//   - Hat (wedge):      w.hat()
//   - Action on vector: X.act(v)
//
// Because manif prefers Cartesian tangent vectors, the tangent types are
// regular Eigen vectors (as exposed by manif). This adapter uses the native
// manif types where possible.
//
// Notes:
//  - If a particular convenience accessor name (e.g., quat(), translation(),
//    rotation(), velocity(), t()) differs in your installed manif version,
//    edit the one line forwarding that accessor accordingly.

#include <Eigen/Core>

namespace lie_odyssey {

// ------------------------------- Base ---------------------------------

// Template parameter: Native = the manif group type (e.g. manif::SO3d,
// manif::SE3d, manif::SE23d, manif::SGal3d, or other manif::Group types).
template <typename Group>
class BaseManif {
public:
  /*Note: 
      manif exposes tangent objects via X.log() and w.exp(); use decltype to
      obtain the tangent type without assuming a precise typedef name.
  */  
    using Native      = Group;
    using Tangent     = typename Group::Tangent;                        // Lie Algebra Tangent space 
    using MatrixType  = decltype(std::declval<Native>().transform());   // Lie Group matrix type
    using TMatrixType = typename Group::Jacobian;                       // Transformation matrix type

    Native g_;

    BaseManif() : g_() {}
    explicit BaseManif(const Native& gg) : g_(gg) {}

    Native native() { return g_; }

    // Exponential / Logarithm
    // manif documents tangent.exp() (w.exp()) and X.log().
    static BaseManif Exp(const Tangent& u) { return BaseManif(u.exp()); }
    static Tangent Log(const BaseManif& X) { return X.g_.log(); }

    // Right Plus/Minus operators (manif documents both operator overloads
    // and named methods like plus(), rplus(), minus(), rminus()).
    // We'll forward to the documented member functions where possible.
    void plus(const Tangent& u) { g_ *= u.exp(); }

    Tangent minus(const BaseManif& X) const {
        // Right minus: documented X - Y or X.rminus(Y) returns tangent.
        // Compute: Log( X^{-1} * this ) or as documented: this->minus(X) semantics.
        // We implement right-minusing: this ⊖ X = Log( X^{-1} ∘ this )
        Native invX = X.g_.inverse();
        Native relative = invX * g_;
        return relative.log(); 
    }

    // Group ops
    BaseManif operator*(const BaseManif& other) const { return BaseManif(g_ * other.g_); }
    BaseManif Inverse() const { return BaseManif(g_.inverse()); }

    // Adjoint
    // manif documents X.adj() as manifold adjoint
    TMatrixType Adjoint() const { return g_.adj(); }
    // manif may not directly document invAdj(), but adj() is invertible; we call
    // inverse on the matrix returned by adj() if available.
    TMatrixType invAdjoint() const { 
        auto A = g_.adj();
        // attempt to use .inverse() on the returned adjoint type; if it is an Eigen
        // matrix-like type this will compile. If not available, user can call
        // native().adj().inverse() or compute differently.
        return A.inverse();
    }
    // Tangent adjoint (tangent.smallAdj())
    TMatrixType adjoint(const Tangent& u) const { return u.smallAdj(); }

    // Matrix representation
    MatrixType asMatrix() const { return g_.transform(); }

    // Jacobians (w.r.t perturbation)
    TMatrixType leftJacobian(const Tangent& u) const { return u.ljac(); }
    TMatrixType invLeftJacobian(const Tangent& u) const { return u.ljacinv(); }
    TMatrixType rightJacobian(const Tangent& u) const { return u.rjac(); }
    TMatrixType invRightJacobian(const Tangent& u) const { return u.rjacinv(); }
};

// ------------------------------- SO(3) ---------------------------------

template <typename Scalar = double>
class SO3Manif : public BaseManif<manif::SO3<Scalar>> {
    using Base = BaseManif<manif::SO3<Scalar>>;
  public:
    SO3Manif() : Base() { }
    explicit SO3Manif(const Base::Native& gg) : Base(gg) { }

    // Convenience accessors (forward to Manif)
    auto q() const { return this->g_.quat(); }      // quaternion
    auto R() const { return this->g_.rotation(); }  // rotation matrix
};

// ------------------------------- SE(3) ---------------------------------

template <typename Scalar = double>
class SE3Manif : public BaseManif<manif::SE3<Scalar>> {
    using Base = BaseManif<manif::SE3<Scalar>>;
  public:
    SE3Manif() : Base() { }
    explicit SE3Manif(const Base::Native& gg) : Base(gg) { }

    // Convenience accessors (forward to Manif)
    auto q() const { return this->g_.quat(); }           // quaternion
    auto R() const { return this->g_.rotation(); }       // rotation matrix
    auto p() const { return this->g_.translation(); }    // position / translation
};

// ------------------------------ SE_2(3) ---------------------------------

template <typename Scalar = double>
class SE23Manif : public BaseManif<manif::SE23<Scalar>> {
    using Base = BaseManif<manif::SE23<Scalar>>;
  public:
    SE23Manif() : Base() { }
    explicit SE23Manif(const Base::Native& gg) : Base(gg) { }

    // Convenience accessors (forward to Manif)
    auto q() const { return this->g_.quat(); }
    auto R() const { return this->g_.rotation(); }
    auto v() const { return this->g_.linearVelocity(); }      
    auto p() const { return this->g_.translation(); }
};

// ------------------------------ SGal(3) / Gal(3) -------------------------

template <typename Scalar = double>
class Gal3Manif : public BaseManif<manif::SGal3<Scalar>> {
    using Base = BaseManif<manif::SGal3<Scalar>>;
  public:
    Gal3Manif() : Base() { }
    explicit Gal3Manif(const Base::Native& gg) : Base(gg) { }

    auto q() const { return this->g_.quat(); }
    auto R() const { return this->g_.rotation(); }
    auto v() const { return this->g_.linearVelocity(); } 
    auto p() const { return this->g_.translation(); }   
    auto t() const { return this->g_.t(); }  // time accessor
};

// ------------------------------- Bundle / Composite ---------------------
//
// manif supports Bundle<> (composite manifold). Users wanting a wrapper for
// Bundle should instantiate BaseManif<manif::Bundle<...>> directly. We don't
// create a dedicated convenience wrapper here because the fields are user-
// defined (the bundle template params).
//
// -----------------------------------------------------------------------

} // namespace lie_odyssey

#endif