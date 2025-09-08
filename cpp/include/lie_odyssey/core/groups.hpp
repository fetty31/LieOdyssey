#ifndef __LIEODYSSEY_CORE_GROUPS_HPP__
#define __LIEODYSSEY_CORE_GROUPS_HPP__

#if LIE_BACKEND_LIEPLUSPLUS
  #include "lie_odyssey/backends/liepp_groups.hpp"
#endif

#if LIE_BACKEND_MANIF
  #include "lie_odyssey/backends/manif_groups.hpp"
#endif

namespace lie_odyssey {

/// Generic Lie group wrapper
template <typename BackendGroup>
class LieGroup {
public:
    using Impl       = BackendGroup;
    using Tangent    = typename Impl::Tangent;
    using MatrixType = typename Impl::MatrixType;
    using Jacobian   = typename Impl::Jacobian;

    LieGroup() = default;
    explicit LieGroup(const Impl& impl) : impl_(impl) { }
    explicit LieGroup(Impl&& impl) : impl_(std::move(impl)) { }
    explicit LieGroup(const typename Impl::Native& native) : impl_(native) { }

    // Identity
    static LieGroup Identity() { 
        return LieGroup(Impl::Identity()); 
    }

    void setIdentity() { 
        impl_.setIdentity(); 
    }

    // Exponential / Logarithm
    static LieGroup Exp(const Tangent& u) { 
        return LieGroup(Impl::Exp(u)); 
    }
    Tangent Log() const { 
        return impl_.Log(); 
    }

    // Group operations
    LieGroup operator*(const LieGroup& other) const { 
        return LieGroup(impl_ * other.impl_); 
    }
    LieGroup Inverse() const { 
        return LieGroup(impl_.Inverse()); 
    }

    // Right plus / minus
    void plus(const Tangent& u) { 
        impl_.plus(u); 
    }

    Tangent minus(const LieGroup& X) const { 
        // X ⊖ this = Log( X^{-1} * this )
        return impl_.minus(X.impl_); 
    }

    // Adjoint
    Jacobian Adjoint() const { 
        return impl_.Adjoint(); 
    }
    Jacobian invAdjoint() const { 
        return impl_.invAdjoint(); 
    }
    Jacobian adjoint(const Tangent& u) const { 
        return impl_.adjoint(u); 
    }

    // Matrix representation
    MatrixType asMatrix() const { 
        return impl_.asMatrix(); 
    }

    // Left / Right Jacobians
    Jacobian leftJacobian(const Tangent& u) const { 
        return impl_.leftJacobian(u); 
    }
    Jacobian invLeftJacobian(const Tangent& u) const { 
        return impl_.invLeftJacobian(u); 
    }
    Jacobian rightJacobian(const Tangent& u) const { 
        return impl_.rightJacobian(u); 
    }
    Jacobian invRightJacobian(const Tangent& u) const { 
        return impl_.invRightJacobian(u); 
    }

    // Access to underlying backend
    const Impl& impl() const { return impl_; }
    Impl& impl() { return impl_; }

private:
    Impl impl_;
};

} // namespace lie_odyssey

#endif
