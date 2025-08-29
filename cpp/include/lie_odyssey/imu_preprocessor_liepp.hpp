#include <iostream>
#include <Eigen/Dense>
#include <groups/Gal.hpp>

using namespace Lie;

template <typename Scalar = double>
class IMUPreintegratorSGal {
public:
    using Group   = Gal3<Scalar>;
    using Tangent = typename Group::Tangent; // 10x1: [rho(3), nu(3), theta(3), s(1)]
    using Vec3    = Eigen::Matrix<Scalar,3,1>;
    using Vec10   = Eigen::Matrix<Scalar,10,1>;
    using Mat3    = Eigen::Matrix<Scalar,3,3>;
    using Mat10   = Eigen::Matrix<Scalar,10,10>;
    using Mat106  = Eigen::Matrix<Scalar,10,6>;
    using Mat6    = Eigen::Matrix<Scalar,6,6>;

    Group dX;                    // preintegrated delta
    Eigen::Matrix<Scalar,10,3> J_bg; // bias gyro
    Eigen::Matrix<Scalar,10,3> J_ba; // bias accel
    Mat10 cov;                    // covariance in tangent
    Mat3 cov_gyro, cov_acc;
    Scalar sum_dt;

    IMUPreintegratorSGal(const Mat3& covg = Mat3::Identity()*1e-5,
                         const Mat3& cova = Mat3::Identity()*1e-3)
        : dX(Group::Identity()), J_bg(Mat10::Zero()), J_ba(Mat10::Zero()),
          cov(Mat10::Zero()), cov_gyro(covg), cov_acc(cova), sum_dt(0) {}

    void reset() {
        dX = Group::Identity();
        J_bg.setZero();
        J_ba.setZero();
        cov.setZero();
        sum_dt = 0;
    }

    void integrate(const Vec3& omega_meas,
                   const Vec3& acc_meas,
                   const Vec3& bg_nominal,
                   const Vec3& ba_nominal,
                   Scalar dt)
    {
        // unbiased measurements
        Vec3 omega_u = omega_meas - bg_nominal;
        Vec3 acc_u   = acc_meas  - ba_nominal;

        // Tangent increment [rho, nu, theta, s]
        Tangent xi;
        xi.setZero();

        // nu: delta-velocity
        xi.segment<3>(3) = acc_u * dt;
        // theta: rotation
        xi.segment<3>(6) = omega_u * dt;
        // s: dt
        xi(9) = dt;
        // rho: displacement contribution
        Vec3 dv = dX.linearVelocity();
        xi.segment<3>(0) = dv * dt + Scalar(0.5) * acc_u * dt * dt;

        // --- Right-plus integration: dX_new = dX * Exp(xi) ---
        dX = dX * Group::Exp(xi); // Lie++ right-plus: multiply by exponential of tangent

        // --- Bias Jacobians ---
        Eigen::Matrix<Scalar,10,3> dxi_dbg = Eigen::Matrix<Scalar,10,3>::Zero();
        Eigen::Matrix<Scalar,10,3> dxi_dba = Eigen::Matrix<Scalar,10,3>::Zero();
        dxi_dbg.block<3,3>(6,0) = -Mat3::Identity() * dt;       // theta
        dxi_dba.block<3,3>(3,0) = -Mat3::Identity() * dt;       // nu
        dxi_dba.block<3,3>(0,0) = -Scalar(0.5) * Mat3::Identity() * dt * dt; // rho

        // Right-plus Jacobian of Exp(xi) w.r.t xi
        Eigen::Matrix<Scalar,10,10> Jr = Group::RightJacobian(xi);

        // Propagate bias Jacobians through chain rule
        J_bg = Jr * dxi_dbg + J_bg; // previous J_bg + new contribution
        J_ba = Jr * dxi_dba + J_ba;

        // --- Covariance propagation ---
        Mat6 Qm = Mat6::Zero();
        Qm.block<3,3>(0,0) = cov_gyro * dt;
        Qm.block<3,3>(3,3) = cov_acc  * dt;

        // Noise mapping from IMU measurement to tangent
        Mat106 G = Mat106::Zero();
        G.block<3,3>(0,3) = Scalar(0.5) * Mat3::Identity() * dt * dt; // rho
        G.block<3,3>(3,3) = Mat3::Identity() * dt;                     // nu
        G.block<3,3>(6,0) = Mat3::Identity() * dt;                     // theta

        cov = Jr * (G * Qm * G.transpose()) * Jr.transpose() + cov; // propagate covariance

        sum_dt += dt;
    }

    // Apply bias correction
    Group getCorrectedDelta(const Vec3& delta_bg, const Vec3& delta_ba) const {
        Tangent corr;
        corr.setZero();
        corr.head<3>() = J_bg.block<3,3>(0,0) * delta_bg + J_ba.block<3,3>(0,0) * delta_ba;
        corr.segment<3>(3) = J_bg.block<3,3>(3,0) * delta_bg + J_ba.block<3,3>(3,0) * delta_ba;
        corr.segment<3>(6) = J_bg.block<3,3>(6,0) * delta_bg + J_ba.block<3,3>(6,0) * delta_ba;
        corr(9) = J_bg(9,0)*delta_bg(0)+J_bg(9,1)*delta_bg(1)+J_bg(9,2)*delta_bg(2)
                + J_ba(9,0)*delta_ba(0)+J_ba(9,1)*delta_ba(1)+J_ba(9,2)*delta_ba(2);

        return dX * Group::Exp(corr); // right-plus correction
    }

    // Accessors
    Eigen::Matrix<Scalar,3,3> rotationMatrix() const { return dX.rotation(); }
    Vec3 velocity() const { return dX.linearVelocity(); }
    Vec3 position() const { return dX.translation(); }
    Scalar time() const { return dX.t(); }
};

int main() {
    using Scalar = double;
    using Vec3 = Eigen::Matrix<Scalar,3,1>;

    IMUPreintegratorSGal<Scalar> preint;

    // Example IMU measurements
    for (int i=0; i<5; ++i) {
        Vec3 omega(0.01*i,0.02*i,0.03*i);
        Vec3 acc(0.1*i,0.2*i,9.81);
        Vec3 bg = Vec3::Zero();
        Vec3 ba = Vec3::Zero();
        preint.integrate(omega, acc, bg, ba, 0.01);
    }

    std::cout << "Velocity: " << preint.velocity().transpose() << "\n";
    return 0;
}
