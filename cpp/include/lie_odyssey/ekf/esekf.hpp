#ifndef __LIEODYSSEY_ESEKF_INS_HPP__
#define __LIEODYSSEY_ESEKF_INS_HPP__

#include "lie_odyssey/ekf/base_filter.hpp"

namespace lie_odyssey {

// -------------------- Right iESEKF for Lie Groups --------------------
template <typename Group>
class ESEKF : public BaseFilter<Group> {
public:

    using Base = BaseFilter<Group>;

    ESEKF(const Vec3& ba = Vec3::Zero(), 
        const Vec3& bg = Vec3::Zero(), 
        const MatDoFext& P = MatDoFext::Identity()*Scalar(1e-3),
        const Mat3& cov_acc_init = Mat3::Identity()*Scalar(1e-5),
        const Mat3& cov_gyro_init = Mat3::Identity()*Scalar(1e-3))
        : Base(ba, bg, P, cov_acc_init, cov_gyro_init);
    { }

};

} // namespace lie_odyssey


#endif // __LIEODYSSEY_ESEKF_INS_HPP__