#include <gtest/gtest.h>
#include <Eigen/Dense>

#include <core/preintegrator.hpp>
#include <core/groups.hpp>

using namespace lie_odyssey;

using Vec3 = Eigen::Vector3d;

template <typename T, typename = void>
struct has_velocity_method : std::false_type {};

template <typename T>
struct has_velocity_method<T, std::void_t<decltype(std::declval<T>().impl().v())>> 
    : std::true_type {};

template <typename T, typename = void>
struct has_position_method : std::false_type {};

template <typename T>
struct has_position_method<T, std::void_t<decltype(std::declval<T>().impl().p())>> 
    : std::true_type {};

// ---------------------- Generic Preintegrator Test Fixture ----------------------
template <typename GroupT>
class PreintegratorTestFixture : public ::testing::Test {
public:
    using PreInt = Preintegrator<GroupT>;

    PreInt pre;

    Vec3 gyro{0.0, 0.0, 0.1};
    Vec3 acc{0.1, 0.0, -9.81};
    Vec3 bg{0.0, 0.0, 0.0};
    Vec3 ba{0.0, 0.0, 0.0};
    double dt{0.01};

    void integrateNSteps(int N, const Vec3& a = Vec3::Zero(), const Vec3& g = Vec3::Zero(),
                         const Vec3& ba_bias = Vec3::Zero(), const Vec3& bg_bias = Vec3::Zero()) {
        Vec3 g_use = g.isZero(1e-12) ? gyro : g;
        Vec3 a_use = a.isZero(1e-12) ? acc : a;
        Vec3 bg_use = bg_bias.isZero(1e-12) ? bg : bg_bias;
        Vec3 ba_use = ba_bias.isZero(1e-12) ? ba : ba_bias;

        for(int i = 0; i < N; ++i)
            pre.integrate(a_use, g_use, ba_use, bg_use, dt);
    }
};

// ---------------------- Test Type List ----------------------
#if LIE_BACKEND_MANIF
  using SGal3d = LieGroup<Gal3Manif<double>>;
  using SO3d   = LieGroup<SO3Manif<double>>;
  using SE3d   = LieGroup<SE3Manif<double>>;
  using SE23d  = LieGroup<SE23Manif<double>>;
#else
  using SGal3d = LieGroup<Gal3LiePP<double>>;
  using SO3d   = LieGroup<SO3LiePP<double>>;
  using SE3d   = LieGroup<SE3LiePP<double>>;
  using SE23d  = LieGroup<SE23LiePP<double>>;
#endif

using LieGroupsToTest = ::testing::Types<SGal3d, SO3d, SE3d, SE23d>;
TYPED_TEST_SUITE(PreintegratorTestFixture, LieGroupsToTest);

// ---------------------- Tests ----------------------
TYPED_TEST(PreintegratorTestFixture, Initialization) {
    auto state = this->pre.getState();
    EXPECT_NEAR(this->pre.getTotalTime(), 0.0, 1e-12);
}

TYPED_TEST(PreintegratorTestFixture, SingleIntegrationStep) {
    this->pre.integrate(this->gyro, this->acc, this->bg, this->ba, this->dt);
    auto state = this->pre.getState();
    EXPECT_NE(this->pre.getTotalTime(), 0.0);

    EXPECT_FALSE(state.impl().R().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    if constexpr (has_velocity_method<TypeParam>::value) {
        EXPECT_FALSE(state.impl().v().isZero(1e-12));
    }
}

TYPED_TEST(PreintegratorTestFixture, MultiStepIntegration) {
    this->integrateNSteps(100);
    auto state = this->pre.getState();
    EXPECT_NEAR(this->pre.getTotalTime(), 1.0, 1e-12); // 100 * 0.01

    EXPECT_FALSE(state.impl().R().isApprox(Eigen::Matrix3d::Identity(), 1e-12));
    if constexpr (has_velocity_method<TypeParam>::value) {
        EXPECT_FALSE(state.impl().v().isZero(1e-12));
    }
}

TYPED_TEST(PreintegratorTestFixture, BiasCorrection) {
    this->integrateNSteps(50);
    auto orig = this->pre.getState();
    Vec3 delta_bg(0.001, 0.002, -0.001);
    Vec3 delta_ba(-0.01, 0.01, -0.02);
    auto corrected = this->pre.getAndCorrectDelta(delta_ba, delta_bg);

    EXPECT_FALSE(corrected.impl().R().isApprox(Eigen::Matrix3d::Identity(), 1e-12));

    // Corrected state should differ from original
    if constexpr (has_velocity_method<TypeParam>::value) {
        EXPECT_FALSE(corrected.impl().v().isApprox(orig.impl().v(), 1e-12));
    }
    if constexpr (has_position_method<TypeParam>::value) {
        EXPECT_FALSE(corrected.impl().p().isApprox(orig.impl().p(), 1e-12));
    }
}

TYPED_TEST(PreintegratorTestFixture, ResetFunction) {
    this->pre.integrate(this->acc, this->gyro, this->ba, this->bg, this->dt);
    this->pre.reset();
    EXPECT_NEAR(this->pre.getTotalTime(), 0.0, 1e-12);
}

// ---------------------- SGal3 Preintegrator Tests ----------------------

#if LIE_BACKEND_MANIF
  using PreInt = Preintegrator<LieGroup<Gal3Manif<double>>>;
#else
  using PreInt = Preintegrator<LieGroup<Gal3LiePP<double>>>;
#endif

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

    pre.integrate(acc, gyro, ba, bg, dt);

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
        pre.integrate(acc, gyro, ba, bg, dt);

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
        pre.integrate(acc, gyro, ba, bg, dt);

    auto orig = pre.getState();

    Vec3 delta_bg(0.001, 0.002, -0.001);
    Vec3 delta_ba(-0.01, 0.01, -0.02);

    auto corrected = pre.getAndCorrectDelta(delta_ba, delta_bg);

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

    pre.integrate(acc, gyro, ba, bg, dt);
    pre.reset();

    auto state = pre.getState();
    Eigen::Matrix<double,5,5> I = Eigen::Matrix<double,5,5>::Identity();
    EXPECT_TRUE(state.impl().asMatrix().isApprox(I, 1e-12));
    EXPECT_EQ(pre.getTotalTime(), 0.0);
}