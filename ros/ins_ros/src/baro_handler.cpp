#include "ins_ros/baro_handler.hpp"

using namespace ins_ros;

void iESEKF::baro::H_fun(const iESEKF::Filter& kf,
                        const iESEKF::Group& X,
                        iESEKF::Measurement& r,
                        iESEKF::HMat& H)
{
    /* To-Do:
        - get barometric measurement (z in filter frame, e.g., ENU)
    */
    double z_m = 0.0; 

    using Scalar = iESEKF::Scalar;
    
    const int DoF = iESEKF::Group::DoF;

    r = iESEKF::Measurement::Zero(1);

    // Extract position
    State::V3 p = X.subgroup<0>().translation();

    r(0) = z_m - p.z(); // residual

    H = iESEKF::HMat::Zero(1, DoF);

    // derivative wrt position z
    H(0, 2) = 1.0;
}
