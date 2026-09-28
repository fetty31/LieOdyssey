#ifndef __LIEODYSSEY_CORE_IMU_DATA_HPP__
#define __LIEODYSSEY_CORE_IMU_DATA_HPP__

namespace lie_odyssey {

template <typename Scalar>
struct IMUbias // IMU bias in base_link/body frame 
{ 
    Eigen::Matrix<Scalar,3,1> gyro;
    Eigen::Matrix<Scalar,3,1> accel;

    IMUbias() : gyro(Eigen::Matrix<Scalar,3,1>::Zero()),
                accel(Eigen::Matrix<Scalar,3,1>::Zero())
                { }
};                    

template <typename Scalar>
struct IMUmeas // IMU measurement
{ 
    double stamp;
    double dt; // defined as the difference between the current and the previous measurement
    Eigen::Matrix<Scalar,3,1> gyro;
    Eigen::Matrix<Scalar,3,1> accel;
    IMUbias<Scalar> bias;

    IMUmeas() : gyro(Eigen::Matrix<Scalar,3,1>::Zero()),
                accel(Eigen::Matrix<Scalar,3,1>::Zero()),
                dt(0.0),
                stamp(-1.0)
                { }
};


} // namespace lie_odyssey

#endif // __LIEODYSSEY_CORE_IMU_DATA_HPP__