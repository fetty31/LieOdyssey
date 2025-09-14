#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <core/preintegrator.hpp>
#include <core/groups.hpp>

using namespace lie_odyssey;

// ---------------------- Preintegrator Tests ----------------------

using PreInt = Preintegrator<LieGroup<Gal3Manif<double>>>;
// using PreInt = Preintegrator<LieGroup<Gal3LiePP<double>>>;
using Vec3   = Eigen::Vector3d;

TEST(PreintegratorTest, Initialization) {
    PreInt pre;
    auto state = pre.getState();

    // Initial state should be identity
    Eigen::Matrix<double,5,5> I = Eigen::Matrix<double,5,5>::Identity();
    EXPECT_TRUE(state.impl().asMatrix().isApprox(I, 1e-12));
    EXPECT_EQ(pre.getTotalTime(), 0.0);
}

TEST(PreintegratorTest, SingleIntegrationStep) {
    PreInt pre;
    Vec3 gyro(0.0, 0.0, 0.1);
    Vec3 acc(0.0, 0.0, -9.81);
    Vec3 bg(0.0,0.0,0.0);
    Vec3 ba(0.0,0.0,0.0);
    double dt = 0.01;

    pre.integrate(gyro, acc, bg, ba, dt);

    auto state = pre.getState();
    EXPECT_NE(pre.getTotalTime(), 0.0);

    // Rotation and velocity should have changed
    EXPECT_FALSE(state.impl().v().isZero(1e-12));
    EXPECT_FALSE(state.impl().R().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
}

TEST(PreintegratorTest, MultiStepIntegration) {
    PreInt pre;
    double dt = 0.01;
    int N = 100;
    Vec3 gyro(0.0, 0.0, 0.05);
    Vec3 acc(0.0, 0.0, -9.81);
    Vec3 bg(0.01, -0.01, 0.005);
    Vec3 ba(0.1, -0.05, 0.02);

    for(int i=0; i<N; ++i)
        pre.integrate(gyro, acc, bg, ba, dt);

    auto state = pre.getState();

    // Check that total time matches
    EXPECT_NEAR(pre.getTotalTime(), dt*N, 1e-12);

    // Check that preintegrated velocity and position are not zero
    EXPECT_FALSE(state.impl().v().isZero(1e-8));
    EXPECT_FALSE(state.impl().p().isZero(1e-8));
}

TEST(PreintegratorTest, BiasCorrection) {
    PreInt pre;
    double dt = 0.01;
    int N = 50;
    Vec3 gyro(0.0, 0.0, 0.1);
    Vec3 acc(0.0, 0.0, -9.81);
    Vec3 bg(0.01, -0.01, 0.005);
    Vec3 ba(0.1, -0.05, 0.02);

    for(int i=0; i<N; ++i)
        pre.integrate(gyro, acc, bg, ba, dt);

    auto orig = pre.getState();

    Vec3 delta_bg(0.001, 0.002, -0.001);
    Vec3 delta_ba(-0.01, 0.01, -0.02);

    auto corrected = pre.getAndCorrectDelta(delta_bg, delta_ba);

    // Corrected state should differ from original
    EXPECT_FALSE(corrected.impl().v().isApprox(orig.impl().v(), 1e-12));
    EXPECT_FALSE(corrected.impl().p().isApprox(orig.impl().p(), 1e-12));
}

TEST(PreintegratorTest, ResetFunction) {
    PreInt pre;
    Vec3 gyro(0.0,0.0,0.1);
    Vec3 acc(0.0,0.0,-9.81);
    Vec3 bg(0.0,0.0,0.0);
    Vec3 ba(0.0,0.0,0.0);
    double dt = 0.01;

    pre.integrate(gyro, acc, bg, ba, dt);
    pre.reset();

    auto state = pre.getState();
    Eigen::Matrix<double,5,5> I = Eigen::Matrix<double,5,5>::Identity();
    EXPECT_TRUE(state.impl().asMatrix().isApprox(I, 1e-12));
    EXPECT_EQ(pre.getTotalTime(), 0.0);
}

// ---------------------- Main ----------------------
int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
