#include <iostream>
#include <vector>

#include "lie_odyssey/lie_odyssey.hpp"

using namespace lie_odyssey;

int main(int argc, char **argv)
{
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    // Noise covariances (spectral densities)
    Mat3 cov_g = Mat3::Identity() * 1e-5; // gyro
    Mat3 cov_a = Mat3::Identity() * 1e-3; // accel

    Preintegrator<LieGroup<Gal3Manif<double>>> pre(cov_g, cov_a);

    // Nominal biases
    Vec3 bg_nom(0.01, -0.02, 0.005);
    Vec3 ba_nom(0.1, -0.05, 0.02);

    // IMU measurement
    IMUmeas imu_meas;
    imu_meas.bias.gyro  = bg_nom;
    imu_meas.bias.accel = ba_nom;

    // Simulate IMU data: 200 Hz, 1 second
    double dt = 0.005;
    int N = 200;
    std::vector<Vec3> gyro_samples(N, Vec3(0,0,0.1)); // rotation around z
    std::vector<Vec3> acc_samples(N, Vec3(0,0,-9.81)); // gravity

    imu_meas.dt = dt;
    for(int k=0; k<N; ++k){
        imu_meas.gyro  = gyro_samples[k]; imu_meas.accel = acc_samples[k];
        pre.integrate(imu_meas);
    }

    auto state = pre.getState();

    std::cout << "Preintegrated Δt: " << pre.getTotalTime() << " s\n";
    std::cout << "ΔR =\n" << state.impl().R() << "\n";
    std::cout << "Δv = " << state.impl().v().transpose() << "\n";
    std::cout << "Δp = " << state.impl().p().transpose() << "\n";

    // Small bias correction example
    Vec3 delta_bg(0.001, 0.001, 0.001);
    Vec3 delta_ba(-0.01, -0.01, -0.01);
    auto dX_corr = pre.getAndCorrectDelta(delta_bg, delta_ba);

    std::cout << "Corrected Δv = " << dX_corr.impl().v().transpose() << "\n";
    std::cout << "Corrected Δp = " << dX_corr.impl().p().transpose() << "\n";

    return 0;
}