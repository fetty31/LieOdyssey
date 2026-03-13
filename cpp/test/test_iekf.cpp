#include <gtest/gtest.h>
#include <lie_odyssey/lie_odyssey.hpp>

using namespace lie_odyssey;

// ---------------------- Generic InvariantEKF Test Fixture ----------------------
template <typename GroupT>
class InvariantEKFTestFixture : public ::testing::Test {
public:
    using Group = GroupT;
    using Filter = InvariantEKF<Group>;
    using Scalar = typename Filter::Scalar;
    static constexpr int DoF = Filter::DoF;

    // Standard RI-EKF uses a 12x12 or DoF-sized noise matrix depending on implementation
    // We'll use the DoF-sized noise for this generic test.
    using NoiseMatrix = Eigen::Matrix<Scalar, 12, 12>; 

    // Simple Invariant Dynamics: constant velocity in the Lie Algebra
    static typename Filter::Tangent simpleDynamics(const Filter& /*f*/, const IMUmeas& /*imu*/) {
        typename Filter::VecTangent t = Filter::VecTangent::Zero();
        t(0) = Scalar(1.0); // Constant angular velocity or velocity along first axis
        return t; 
    }

    // RI-EKF Jacobian f_dx (The 'A' matrix of error dynamics)
    static typename Filter::Jacobian simpleJacX(const Filter& /*f*/, const IMUmeas& /*imu*/) {
        // In RI-EKF, A is often zero or dependent on inputs, but never on R or p
        return Filter::Jacobian::Zero();
    }

    // RI-EKF Noise Mapping f_dw
    static typename Filter::MappingMatrix simpleJacW(const Filter& /*f*/, const IMUmeas& /*imu*/) {
        typename Filter::MappingMatrix W = Filter::MappingMatrix::Zero();
        W.template block<DoF, DoF>(0,0).setIdentity(); // Map first 12 noises to DoF
        return W;
    }

    // Basic members available to tests
    Filter filter{Filter::MatDoF::Identity() * Scalar(1e-3),
                  NoiseMatrix::Identity() * Scalar(1e-3),
                  simpleDynamics,
                  simpleJacX,
                  simpleJacW};

    IMUmeas imu;
    double dt{0.01};

    InvariantEKFTestFixture() {
        imu.dt = dt;
    }
};

// ---------------------- Test Type List ----------------------
#if LIE_BACKEND_MANIF
  using SO3d   = LieGroup<SO3Manif<double>>;
  using SE3d   = LieGroup<SE3Manif<double>>;
  using SE23d  = LieGroup<SE23Manif<double>>;
#else
  using SO3d   = LieGroup<SO3LiePP<double>>;
  using SE3d   = LieGroup<SE3LiePP<double>>;
  using SE23d  = LieGroup<SE23LiePP<double>>;
#endif

// Testing common Navigation groups
using LieGroupsToTest = ::testing::Types<SO3d, SE3d, SE23d>;
TYPED_TEST_SUITE(InvariantEKFTestFixture, LieGroupsToTest);

// ---------------------- Tests ----------------------

TYPED_TEST(InvariantEKFTestFixture, PredictMaintainsInvariantProperty) {
    auto state_before = this->filter.getState();
    
    // Predict
    this->filter.predict(this->imu);
    
    auto state_after = this->filter.getState();
    
    // In RI-EKF, prediction is X = X * exp(u*dt)
    // We check that the state has actually moved
    auto dx = state_after.minus(state_before); 
    
    #if LIE_BACKEND_MANIF
    EXPECT_GT(dx.coeffs().norm(), 0.0);
    #else
    EXPECT_GT(dx.norm(), 0.0);
    #endif
}

TYPED_TEST(InvariantEKFTestFixture, RightInvariantUpdateStep) {
    using Measurement = Eigen::Matrix<double, 3, 1>;
    using HMat = Eigen::Matrix<double, 3, TestFixture::DoF>;

    // Setup a non-zero covariance
    this->filter.setCovariance(TestFixture::Filter::MatDoF::Identity() * 0.1);

    // Simulated Observation (e.g., world-frame position or vector)
    Measurement y = Measurement::Zero();
    y(2) = 1.0; 

    // Innovation for RI-EKF: r = \hat{X} * y_obs - b_fixed
    auto h_fun = [](const typename TestFixture::Filter& /*f*/, 
                    const typename TestFixture::Group& X, 
                    const Measurement& obs) -> Measurement {
        // Simple observation: identity map for testing
        return Measurement::Zero(); 
    };

    auto H_fun = [](const typename TestFixture::Filter& /*f*/, 
                    const typename TestFixture::Group& /*X*/) -> HMat {
        HMat H = HMat::Zero();
        H.template block<3,3>(0,0).setIdentity(); // Observe first 3 states
        return H;
    };

    auto P_before = this->filter.getCovariance();

    // Perform Update
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.01;
    this->filter.template update<Measurement, HMat>(
        y, R, R.inverse(), h_fun, H_fun
    );

    auto P_after = this->filter.getCovariance();

    // Covariance should decrease after a measurement update
    EXPECT_LT(P_after.trace(), P_before.trace());
}

TYPED_TEST(InvariantEKFTestFixture, ConsistencyUnderLargeRotation) {
    // One of the main strengths of RI-EKF is consistency regardless of heading.
    // We test that an update produces the same covariance reduction 
    // regardless of the initial orientation.
    
    using Measurement = Eigen::Matrix<double, 3, 1>;
    using HMat = Eigen::Matrix<double, 3, TestFixture::DoF>;
    
    auto h_fun = [](const auto&, const auto&, const auto& y) { return y; };
    auto H_fun = [](const auto&, const auto&) { 
        HMat H = HMat::Zero(); 
        H.template block<3,3>(0,0).setIdentity(); 
        return H; 
    };
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() * 0.01;

    // 1. Update at Identity
    this->filter.reset();
    this->filter.setCovariance(TestFixture::Filter::MatDoF::Identity() * 0.1);
    this->filter.template update<Measurement, HMat>(Measurement::Zero(), R, R.inverse(), h_fun, H_fun);
    double trace_identity = this->filter.getCovariance().trace();

    // 2. Update at 90-degree rotation
    this->filter.reset();
    typename TestFixture::Group rotated_state;
    typename TestFixture::Group::Tangent rot_vec = TestFixture::Group::Tangent::Zero();
    rot_vec(2) = 1.57; // 90 deg Z
    rotated_state.plus(rot_vec);
    
    this->filter.setState(rotated_state);
    this->filter.setCovariance(TestFixture::Filter::MatDoF::Identity() * 0.1);
    this->filter.template update<Measurement, HMat>(Measurement::Zero(), R, R.inverse(), h_fun, H_fun);
    double trace_rotated = this->filter.getCovariance().trace();

    // RI-EKF property: Covariance update is independent of the state (for linear H in invariant frame)
    EXPECT_NEAR(trace_identity, trace_rotated, 1e-10);
}

TYPED_TEST(InvariantEKFTestFixture, ResetFunctionality) {
    this->filter.predict(this->imu);
    this->filter.setCovariance(TestFixture::Filter::MatDoF::Identity() * 5.0);
    
    this->filter.reset();
    
    auto P = this->filter.getCovariance();
    EXPECT_NEAR(P(0,0), 1e-3, 1e-12);
    
    // Check state is back to Identity (using Log to check distance to identity)
    auto log_val = this->filter.getState().Log();
    #if LIE_BACKEND_MANIF
    EXPECT_NEAR(log_val.coeffs().norm(), 0.0, 1e-12);
    #else
    EXPECT_NEAR(log_val.norm(), 0.0, 1e-12);
    #endif
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}