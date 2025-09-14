#ifndef __LIEODYSSEY_CORE_IMU_DATA_HPP__
#define __LIEODYSSEY_CORE_IMU_DATA_HPP__

namespace lie_odyssey {

struct IMUbias // IMU bias in base_link/body frame 
{ 
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;

    IMUbias() : gyro(Eigen::Vector3d::Zero()),
                accel(Eigen::Vector3d::Zero())
                { }
};                    

struct IMUmeas // IMU measurement
{ 
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3d gyro;
    Eigen::Vector3d accel;
    IMUbias bias;

    IMUmeas() : gyro(Eigen::Vector3d::Zero()),
                accel(Eigen::Vector3d::Zero()),
                dt(0.0),
                stamp(-1.0)
                { }
};


} // namespace lie_odyssey

#endif