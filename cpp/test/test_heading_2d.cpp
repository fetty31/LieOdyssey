#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <cmath>

#include <lie_odyssey/lie_odyssey.hpp>
#include "ins_ros/ekf.hpp"
#include "ins_ros/sensors/yaw_handler.hpp"

using namespace ins_ros::iESEKF;

using Scalar = ins_ros::State::Scalar;
using Vec3 = Eigen::Matrix<Scalar, 3, 1>;
using Quat = Eigen::Quaternion<Scalar>;
using Vec2 = Eigen::Matrix<Scalar, 2, 1>;

using Bundle = ins_ros::iESEKF::Bundle;
using Group = ins_ros::iESEKF::Group;
using Filter = ins_ros::iESEKF::Filter;
using IMUmeas = ins_ros::iESEKF::IMUmeas;
using Measurement = ins_ros::iESEKF::Measurement;
using HMat = ins_ros::iESEKF::HMat;
using MatDoF = ins_ros::iESEKF::MatDoF;
using NoiseMatrix = Filter::NoiseMatrix;

// ---------------------- Helper Functions ----------------------

Group make_group(const Vec3& p, const Quat& q, const Vec3& v,
                 const Vec3& bg, const Vec3& ba, const Vec3& g, double t) {
    using SGal3 = manif::SGal3<Scalar>;
    using R3 = manif::R3<Scalar>;
    using NativeBundle = manif::Bundle<Scalar, manif::SGal3, manif::R3, manif::R3, manif::R3>;
    NativeBundle native(SGal3(p, q, v, t), R3(bg), R3(ba), R3(g));
    return Group(Bundle(native));
}

// Skew-symmetric matrix helper
Eigen::Matrix<Scalar, 3, 3> my_skew(const Vec3& v) {
    Eigen::Matrix<Scalar, 3, 3> S;
    S << Scalar(0), -v(2), v(1),
         v(2), Scalar(0), -v(0),
         -v(1), v(0), Scalar(0);
    return S;
}

// Dynamics function
Filter::Tangent dynamics(const Filter& kf, const IMUmeas& imu) {
    Filter::VecTangent t = Filter::VecTangent::Zero();
    Group X = kf.getState();
    auto g = X.impl().subgroup<3>().coeffs();
    auto R = X.impl().subgroup<0>().quat().toRotationMatrix();
    auto b_a = X.impl().subgroup<2>().coeffs();
    auto b_w = X.impl().subgroup<1>().coeffs();

    t.template segment<3>(3) = (imu.accel - b_a).cast<Scalar>() - R.transpose() * g;
    t.template segment<3>(6) = (imu.gyro - b_w).cast<Scalar>();
    t(9) = Scalar(1);
    return t;
}

Filter::Jacobian jacobian_x(const Filter& kf, const IMUmeas& /*imu*/) {
    Filter::Jacobian Jx = Filter::Jacobian::Zero();
    Group X = kf.getState();
    auto g = X.impl().subgroup<3>().coeffs();
    auto R = X.impl().subgroup<0>().quat().toRotationMatrix();

    Jx.block<3, 3>(3, 6) = -my_skew(R.transpose() * g);
    Jx.block<3, 3>(3, 13) = -Eigen::Matrix<Scalar, 3, 3>::Identity();
    Jx.block<3, 3>(3, 16) = -R.transpose();
    Jx.block<3, 3>(6, 10) = -Eigen::Matrix<Scalar, 3, 3>::Identity();
    return Jx;
}

Filter::MappingMatrix jacobian_w(const Filter& /*kf*/, const IMUmeas& /*imu*/) {
    Filter::MappingMatrix Jw = Filter::MappingMatrix::Zero();
    Jw.block<3, 3>(3, 3)  = -Eigen::Matrix<Scalar, 3, 3>::Identity();
    Jw.block<3, 3>(6, 0)  = -Eigen::Matrix<Scalar, 3, 3>::Identity();
    Jw.block<3, 3>(10, 6) =  Eigen::Matrix<Scalar, 3, 3>::Identity();
    Jw.block<3, 3>(13, 9) =  Eigen::Matrix<Scalar, 3, 3>::Identity();
    return Jw;
}

IMUmeas make_imu(const Vec3& gyro, const Vec3& accel, double dt,
                 const Vec3& bg = Vec3::Zero(), const Vec3& ba = Vec3::Zero()) {
    IMUmeas imu;
    imu.gyro = gyro;
    imu.accel = accel;
    imu.dt = dt;
    imu.bias.gyro = bg;
    imu.bias.accel = ba;
    imu.stamp = 0.0;
    return imu;
}

// ---------------------- Test Fixture ----------------------
class Heading2DMeasurementTest : public ::testing::Test {
protected:
    Filter filter;
    IMUmeas imu;
    double dt{0.01};

    Vec3 gt_pos{1.0, 0.0, 0.0};
    double yaw_gt{0.5};
    Quat gt_quat{Eigen::AngleAxisd(yaw_gt, Vec3::UnitZ())};
    Vec3 gt_vel{0.1, 0.0, 0.0};
    Vec3 gt_bg{0.0, 0.0, 0.0};
    Vec3 gt_ba{0.0, 0.0, 0.0};
    Vec3 gt_g{0.0, 0.0, 9.81};
    double gt_time{0.0};

    Heading2DMeasurementTest()
        : filter(MatDoF::Identity() * Scalar(1e-3),
                 NoiseMatrix::Identity() * Scalar(1e-3),
                 dynamics, jacobian_x, jacobian_w)
    {
        imu = make_imu(Vec3(0.0, 0.0, 0.01), Vec3(0.0, 0.0, -9.81), dt, gt_bg, gt_ba);
        filter.setMaxIters(3);
        filter.setTolerance(Scalar(1e-9));
    }

    void SetUp() override {
        Group gt_state = make_group(gt_pos, gt_quat, gt_vel, gt_bg, gt_ba, gt_g, gt_time);
        filter.setState(gt_state);
        filter.setCovariance(MatDoF::Identity() * Scalar(1e-3));
    }
};

// ---------------------- 2D Heading Tests ----------------------

TEST_F(Heading2DMeasurementTest, GroundTruthHeadingMatches) {
    Scalar yaw_meas = yaw_gt;
    Eigen::Matrix<Scalar,2,2> R = Eigen::Matrix<Scalar,2,2>::Zero();
    R.diagonal() = Vec2::Constant(Scalar(0.01));
    auto R_inv = R.inverse();

    auto state_before = filter.getState();

    std::cout << "state_before: " << std::endl;

    filter.update<Scalar, Measurement, HMat>(yaw_meas, R, R_inv, ins_ros::iESEKF::yaw::H_fun);
    auto state_after = filter.getState();

    auto dx = state_after.minus(state_before);
    EXPECT_NEAR(dx.coeffs().norm(), 0.0, 1e-6);
}

TEST_F(Heading2DMeasurementTest, SmallNoiseHeadingProducesSmallCorrection) {
    Scalar noise = 0.01;
    Scalar yaw_meas = yaw_gt + noise;

    Eigen::Matrix<Scalar,2,2> R = Eigen::Matrix<Scalar,2,2>::Zero();
    R.diagonal() = Vec2::Constant(Scalar(0.01));
    auto R_inv = R.inverse();

    auto state_before = filter.getState();
    auto P_before = filter.getCovariance();

    filter.update<Scalar, Measurement, HMat>(yaw_meas, R, R_inv, ins_ros::iESEKF::yaw::H_fun);

    auto state_after = filter.getState();
    auto P_after = filter.getCovariance();

    auto dx = state_after.minus(state_before);
    EXPECT_GT(dx.coeffs().norm(), 0.0);
    EXPECT_LT(dx.coeffs().norm(), 0.5);
    EXPECT_LT(P_after.trace(), P_before.trace());
}

TEST_F(Heading2DMeasurementTest, LargeHeadingNoiseProducesLargerCorrection) {
    Scalar noise = 0.5;
    Scalar yaw_meas = yaw_gt + noise;

    Eigen::Matrix<Scalar,2,2> R = Eigen::Matrix<Scalar,2,2>::Zero();
    R.diagonal() = Vec2::Constant(Scalar(0.001));
    auto R_inv = R.inverse();

    auto state_before = filter.getState();
    filter.update<Scalar, Measurement, HMat>(yaw_meas, R, R_inv, ins_ros::iESEKF::yaw::H_fun);
    auto state_after = filter.getState();

    auto dx = state_after.minus(state_before);
    EXPECT_GT(dx.coeffs().norm(), 0.0);
}

TEST_F(Heading2DMeasurementTest, HeadingCovarianceReduction) {
    Scalar yaw_meas = yaw_gt;
    Eigen::Matrix<Scalar,2,2> R = Eigen::Matrix<Scalar,2,2>::Zero();
    R.diagonal() = Vec2::Constant(Scalar(0.01));
    auto R_inv = R.inverse();

    double prior_trace = filter.getCovariance().trace();
    filter.update<Scalar, Measurement, HMat>(yaw_meas, R, R_inv, ins_ros::iESEKF::yaw::H_fun);
    double post_trace = filter.getCovariance().trace();

    EXPECT_LE(post_trace + 1e-12, prior_trace);
}

TEST_F(Heading2DMeasurementTest, PredictMaintainsStateStructure) {
    Group before = filter.getState();
    filter.predict(imu);
    Group after = filter.getState();

    auto dx = after.minus(before);
    EXPECT_GT(dx.coeffs().norm(), 0.0);
}

// ---------------------- Identity Orientation Heading Test ----------------------

TEST(Heading2DTest, IdentityOrientationGroundTruth) {
    using SGal3 = manif::SGal3<Scalar>;
    using R3 = manif::R3<Scalar>;
    using NativeBundle = manif::Bundle<Scalar, manif::SGal3, manif::R3, manif::R3, manif::R3>;

    Scalar yaw_gt = 0.0;
    Quat gt_quat{Quat::Identity()};
    Vec3 gt_pos{0.0, 0.0, 0.0};
    Vec3 gt_vel{0.0, 0.0, 0.0};
    Vec3 gt_bg{0.0, 0.0, 0.0};
    Vec3 gt_ba{0.0, 0.0, 0.0};
    Vec3 gt_g{0.0, 0.0, 9.81};

    NativeBundle native(SGal3(gt_pos, gt_quat, gt_vel, 0.0), R3(gt_bg), R3(gt_ba), R3(gt_g));
    Group state = Group(Bundle(native));

    Filter filter(MatDoF::Identity() * Scalar(1e-3),
                  NoiseMatrix::Identity() * Scalar(1e-3),
                  dynamics, jacobian_x, jacobian_w);
    filter.setState(state);
    filter.setCovariance(MatDoF::Identity() * Scalar(1e-3));
    filter.setMaxIters(3);
    filter.setTolerance(Scalar(1e-9));

    Scalar yaw_meas = 0.0;
    Eigen::Matrix<Scalar,2,2> R = Eigen::Matrix<Scalar,2,2>::Zero();
    R.diagonal() = Vec2::Constant(Scalar(0.01));
    auto R_inv = R.inverse();

    auto state_before = filter.getState();
    filter.update<Scalar, Measurement, HMat>(yaw_meas, R, R_inv, ins_ros::iESEKF::yaw::H_fun);
    auto state_after = filter.getState();

    auto dx = state_after.minus(state_before);
    EXPECT_NEAR(dx.coeffs().norm(), 0.0, 1e-6);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}