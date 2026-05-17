#include "ins_ros/gps_handler.hpp"

using namespace ins_ros;

void iESEKF::gps::H_fun(const iESEKF::Filter& kf,
                        const iESEKF::Group& X,
                        iESEKF::Measurement& r,
                        iESEKF::HMat& H)
{
    /* To-Do:
        - get gps measurement in filter frame (e.g., ENU)
    */
    static Eigen::Vector3d gps_measurement = Eigen::Vector3d(0.0, 0.0, 0.0); // example GPS measurement

    using Scalar = iESEKF::Scalar;
    
    const int DoF = iESEKF::Group::DoF;

    // Measurement: position only (3D GPS fix)
    r = iESEKF::Measurement::Zero(3);

    // Extract position
    State::V3 p = X.subgroup<0>().translation();

    r.segment<3>(0) = gps_measurement - p;

    // Jacobian
    H = iESEKF::HMat::Zero(3, DoF);

    // dp/dp = Identity
    H.block<3,3>(0, 0) = Eigen::Matrix<Scalar, 3, 3>::Identity(); 
}
