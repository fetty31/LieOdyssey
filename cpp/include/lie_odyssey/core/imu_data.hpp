#ifndef __LIEODYSSEY_CORE_IMU_DATA_HPP__
#define __LIEODYSSEY_CORE_IMU_DATA_HPP__

namespace lie_odyssey {

struct IMUbias // IMU bias in base_link/body frame 
{ 
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
};                    

struct IMUmeas // IMU measurement
{ 
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Vector3f gyro;
    Eigen::Vector3f accel;
    IMUbias bias;
};


} // namespace lie_odyssey

#endif