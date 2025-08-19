#pragma once
#include "config.hpp"

#if LIE_BACKEND == LIE_BACKEND_LIEPP
  #include "backends/liepp_groups.hpp"
#elif LIE_BACKEND == LIE_BACKEND_MANIF
  #include "backends/manif_groups.hpp"
#elif LIE_BACKEND == LIE_BACKEND_SOPHUS
  #include "backends/sophus_groups.hpp"
#endif

namespace lie_odyssey {

/// Generic Lie group wrapper
template <typename BackendGroup>
class LieGroup {
public:
    using Impl = BackendGroup;

    LieGroup() = default;
    explicit LieGroup(const Impl& impl) : impl_(impl) {}

    static LieGroup Exp(const Eigen::VectorXd& v) { return LieGroup(Impl::Exp(v)); }
    Eigen::VectorXd Log() const { return impl_.Log(); }

    LieGroup operator*(const LieGroup& other) const { return LieGroup(impl_ * other.impl_); }
    LieGroup Inverse() const { return LieGroup(impl_.Inverse()); }

    Eigen::MatrixXd Adjoint() const { return impl_.Adjoint(); }

    const Impl& impl() const { return impl_; }
    Impl& impl() { return impl_; }

private:
    Impl impl_;
};

} // namespace lie_odyssey
