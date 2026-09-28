#include <iostream>

#include <lie_odyssey/lie_odyssey.hpp>

using namespace lie_odyssey;

// Helper to create skew-symmetric matrix
Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
    return m;
}

int main() {
    using SE23 = SE23LiePP<double>;
    using Group = LieGroup<SE23>;
    using Filter = InvariantEKF<Group>;

    // Define Dynamics
    auto f_dynamics = [](const Filter& f, const IMUmeas& imu) {
        // 1. Correct IMU measurements with current bias estimates
        // (Assuming biases are part of an augmented state or handled externally)
        Eigen::Vector3d omega_corr = imu.gyro; // - bg
        Eigen::Vector3d acc_corr   = imu.accel;  // - ba

        // 2. Define the Tangent vector (the "velocity" in the Lie Algebra)
        // For SE2(3), the tangent vector is [omega, acceleration, velocity]
        // Note: We use the local acceleration but the filter expects the 
        // derivative of the group element.
        Filter::Tangent dot;
        
        // In Lie++ implementation of SE2(3), the tangent order is:
        // [0-2: rotation, 3-5: velocity, 6-8: position]
        // Note: keep in mind that tangent and group order is backend-specific (manif/Lie++)
        dot.segment<3>(0) = omega_corr;
        dot.segment<3>(3) = acc_corr;
        dot.segment<3>(6) = f.getState().impl().v(); // Position derivative is velocity
        
        return dot;
    };

    // Define Diff Dynamics (A matrix for RI-EKF)
    Filter::JacobianXFun f_dx = [](const Filter& f, const IMUmeas& imu) {
        Filter::Jacobian A = Filter::Jacobian::Zero();
        Eigen::Vector3d w = imu.gyro;
        Eigen::Vector3d a = imu.accel;

        // RI-EKF Error Dynamics Matrix A for SE2(3)
        // Order: [Rotation, Velocity, Position]
        A.block<3,3>(0,0) = -skew(w);
        A.block<3,3>(3,0) = -skew(a);
        A.block<3,3>(3,3) = -skew(w);
        A.block<3,3>(6,3) = Eigen::Matrix3d::Identity();
        A.block<3,3>(6,6) = -skew(w);
        return A;
    };

    // Define Noise Mapping (simplified)
    Filter::JacobianWFun f_dw = [](const Filter& f, const IMUmeas& imu) {
        Filter::MappingMatrix W = Filter::MappingMatrix::Zero();
        W.setIdentity(); // Assuming noise is already in the invariant frame
        return W;
    };

    // Initialize Filter
    Filter::MatDoF P = Filter::MatDoF::Identity() * 0.1;
    Filter::NoiseMatrix Q = Filter::NoiseMatrix::Identity() * 0.01;
    
    Filter riekf(P, Q, f_dynamics, f_dx, f_dw);

    // Simulated "Real" Use Case: Constant Velocity Circular Motion
    double dt = 0.1;
    IMUmeas imu;
    imu.dt = dt;
    imu.gyro = Eigen::Vector3d(0, 0, 0.5); // Rotating around Z
    imu.accel  = Eigen::Vector3d(0, 1.0, 0);  // Centripetal acceleration

    std::cout << "Starting RI-EKF Estimation..." << std::endl;

    for (int k = 0; k < 50; ++k) {
        // --- Prediction Step ---
        riekf.predict(imu);

        // --- Measurement Step (GPS Position at k=10, 20, 30...) ---
        if (k % 10 == 0 && k > 0) {
            // Simulated GPS: we observe position in world frame
            // In RI-EKF for SE2(3), observation is: y = R^T * (p_world - p)
            // But usually, we simplify to the Invariant Innovation: r = R*y_obs - (p_world - p)
            
            Eigen::Vector3d gps_pos = riekf.getState().impl().p() + Eigen::Vector3d::Random() * 0.02;
            
            auto h_fun = [gps_pos](const Filter& f, const Group& X, const Eigen::Vector3d& y) -> Eigen::Vector3d {
                // Residual r = X * y - b
                // For position: r = R*y + p - d_world
                // If y is 0 (body frame origin), r = p - d_world
                return X.impl().p() - gps_pos; 
            };

            auto H_fun = [](const Filter& f, const Group& X) -> Eigen::Matrix<double, 3, 9> {
                Eigen::Matrix<double, 3, 9> H = Eigen::Matrix<double, 3, 9>::Zero();
                // Jacobian of position w.r.t RI error: [ [p]x  0  I ]
                H.block<3,3>(0,0) = skew(X.impl().p());
                H.block<3,3>(0,6) = Eigen::Matrix3d::Identity();
                return H;
            };

            Eigen::Matrix3d R_noise = Eigen::Matrix3d::Identity() * 0.01;
            riekf.update<Eigen::Vector3d, Eigen::Matrix<double, 3, 9>>(
                gps_pos, R_noise, R_noise.inverse(), h_fun, H_fun
            );

            std::cout << "K=" << k << " | Updated Pos: " << riekf.getState().impl().p().transpose() << std::endl;
        }
    }

    std::cout << "\nFinal State Estimate:" << std::endl;
    std::cout << "Position: " << riekf.getState().impl().p().transpose() << std::endl;
    std::cout << "Velocity: " << riekf.getState().impl().v().transpose() << std::endl;

    return 0;
}