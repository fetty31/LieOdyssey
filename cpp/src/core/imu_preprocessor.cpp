#include <iostream>
#include <vector>

#include "lie_odyssey/lie_odyssey.hpp"

using namespace lie_odyssey;

int main() {
    using Vec3 = Eigen::Vector3d;
    using Mat3 = Eigen::Matrix3d;

    // Noise covariances (spectral densities)
    Mat3 cov_g = Mat3::Identity() * 1e-5; // gyro
    Mat3 cov_a = Mat3::Identity() * 1e-3; // accel

    ImuPreintegratorSGal<double> pre(cov_g, cov_a);

    // Nominal biases
    Vec3 bg_nom(0.01, -0.02, 0.005);
    Vec3 ba_nom(0.1, -0.05, 0.02);

    // Simulate IMU data: 200 Hz, 1 second
    double dt = 0.005;
    int N = 200;
    std::vector<Vec3> gyro_samples(N, Vec3(0,0,0.1)); // rotation around z
    std::vector<Vec3> acc_samples(N, Vec3(0,0,-9.81)); // gravity

    for(int k=0; k<N; ++k)
        pre.integrate(gyro_samples[k], acc_samples[k], bg_nom, ba_nom, dt);

    std::cout << "Preintegrated Δt: " << pre.sum_dt << " s\n";
    std::cout << "ΔR =\n" << pre.rotationMatrix() << "\n";
    std::cout << "Δv = " << pre.velocity().transpose() << "\n";
    std::cout << "Δp = " << pre.position().transpose() << "\n";

    // Small bias correction example
    Vec3 delta_bg(0.001, 0.001, 0.001);
    Vec3 delta_ba(-0.01, -0.01, -0.01);
    auto dX_corr = pre.getCorrectedDelta(delta_bg, delta_ba);

    std::cout << "Corrected Δv = " << dX_corr.linearVelocity().transpose() << "\n";
    std::cout << "Corrected Δp = " << dX_corr.translation().transpose() << "\n";

    return 0;
}
