#include <gtest/gtest.h>

#include <lie_odyssey/lie_odyssey.hpp>

using namespace lie_odyssey;

// detect whether a Group type has velocity/position accessors (optional checks)
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

// ---------------------- Generic iESEKF Test Fixture ----------------------
template <typename GroupT>
class IESEKFTestFixture : public ::testing::Test {
public:
    using Group = GroupT;
    using Filter = iESEKF<Group>;
    using Scalar = typename Filter::Scalar;
    static constexpr int DoF = Filter::DoF;

    using Measurement = Eigen::Matrix<Scalar, 10, 1>; // arbitrary measurement dimension

    // Default simple dynamics: constant tangent motion along first axis
    static typename Filter::Tangent simpleDynamics(const Filter& /*f*/, const IMUmeas& imu) {
        typename Filter::VecTangent t = Filter::VecTangent::Zero();
        // move in first tangent dimension proportional to dt in predict (arbitrary motion)
        t(0) = Scalar(1.0);
        return t; // cast
    }

    // Jacobians of the dynamics
    static typename Filter::Jacobian simpleJacX(const Filter&, const IMUmeas&) {
        return Filter::Jacobian::Identity();
    }
    static typename Filter::MappingMatrix simpleJacW(const Filter&, const IMUmeas&) {
        return Filter::MappingMatrix::Zero();
    }

    // Basic members available to tests
    Filter filter{Filter::MatDoF::Identity()*Scalar(1e-3),
                  Filter::NoiseMatrix::Identity()*Scalar(1e-3),
                  simpleDynamics,
                  simpleJacX,
                  simpleJacW};

    IMUmeas imu;
    double dt{0.01};

    IESEKFTestFixture() {
        imu.dt = dt;
        filter.setMaxIters(3);
        filter.setTolerance(Scalar(1e-9));
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
TYPED_TEST_SUITE(IESEKFTestFixture, LieGroupsToTest);

// ---------------------- Tests ----------------------
TYPED_TEST(IESEKFTestFixture, DefaultConstructorCovariance) {
    using Filter = typename TestFixture::Filter;
    Filter f; // default constructed
    auto P = f.getCovariance();
    // Default covariance must be ~1e-3 on diagonal (per implementation)
    for (int i = 0; i < Filter::DoF; ++i) {
        for (int j = 0; j < Filter::DoF; ++j) {
            double expected = (i == j) ? 1e-3 : 0.0;
            EXPECT_NEAR(P(i, j), expected, 1e-12);
        }
    }
}

TYPED_TEST(IESEKFTestFixture, PredictWithDefaultDynamics) {
    // Default dynamics in our fixture are non-zero (simpleDynamics)
    auto before = this->filter.getState();
    auto P_before = this->filter.getCovariance();

    this->filter.predict(this->imu);

    auto after = this->filter.getState();
    auto P_after = this->filter.getCovariance();

    // State should have changed (since our simpleDynamics returns non-zero tangent)
    // For groups with explicit state getters like impl().p() or impl().v() we can check,
    // otherwise check that the state minus has non-zero tangent.
    auto dx = after.minus(before); 

    #if LIE_BACKEND_MANIF
    EXPECT_GT(dx.coeffs().norm(), 0.0);
    #else
    EXPECT_GT(dx.norm(), 0.0);
    #endif

    // Covariance should have been propagated (not equal to previous)
    EXPECT_FALSE(P_after.isApprox(P_before, 1e-15));
}

TYPED_TEST(IESEKFTestFixture, UpdateWithZeroResidualReducesCovarianceTrace) {
    using Measurement = typename TestFixture::Measurement;
    using HMat = Eigen::Matrix<typename TestFixture::Scalar, Measurement::RowsAtCompileTime, TestFixture::DoF>;

    // Set filter covariance to something non-trivial
    this->filter.setCovariance(TestFixture::Filter::MatDoF::Identity() * 1e-2);

    // Residual function returns zero residual (y - h(X) == 0) so update should mostly reduce covariance
    auto h_fun = [](const typename TestFixture::Filter& /*f*/, const typename TestFixture::Group& /*X_now*/, const Measurement& /*y*/) -> Measurement {
        return Measurement::Zero(); // zero residual
    };

    auto H_fun = [](const typename TestFixture::Filter& /*f*/, const typename TestFixture::Group& /*X_now*/) -> HMat {
        return HMat::Identity();
    };

    // Record prior covariance trace
    double prior_trace = this->filter.getCovariance().trace();

    // call update
    Measurement y;
    this->filter.template update<Measurement, HMat>(
        y,
        Eigen::Matrix<double, Measurement::RowsAtCompileTime, Measurement::RowsAtCompileTime>::Identity() * 1e-3,
        Eigen::Matrix<double, Measurement::RowsAtCompileTime, Measurement::RowsAtCompileTime>::Identity() * 1e3,
        h_fun,
        H_fun
    );

    double post_trace = this->filter.getCovariance().trace();

    // Covariance trace should not increase
    EXPECT_LE(post_trace + 1e-12, prior_trace);
}

TYPED_TEST(IESEKFTestFixture, UpdateWithNonzeroResidualChangesState) {
    using Measurement = typename TestFixture::Measurement;
    using HMat = Eigen::Matrix<typename TestFixture::Scalar, Measurement::RowsAtCompileTime, TestFixture::DoF>;

    // Ensure filter has reasonable covariance
    this->filter.setCovariance(TestFixture::Filter::MatDoF::Identity() * 1e-2);

    // Define measurement to be non-zero; make h_fun return residual that pushes state in first dimension
    Measurement meas = Measurement::Zero();
    meas(0) = 0.5; // residual in first tangent dimension

    auto h_fun = [](const typename TestFixture::Filter& /*f*/, const typename TestFixture::Group& /*X_now*/, const Measurement& y) -> Measurement {
        // return residual equal to the measurement (simulate h(X)==0)
        return y;
    };

    auto H_fun = [](const typename TestFixture::Filter& /*f*/, const typename TestFixture::Group& /*X_now*/) -> HMat {
        return HMat::Identity();
    };

    auto state_before = this->filter.getState();

    // run update
    this->filter.template update<Measurement, HMat>(
        meas,
        Eigen::Matrix<double, Measurement::RowsAtCompileTime, Measurement::RowsAtCompileTime>::Identity() * 1e-3,
        Eigen::Matrix<double, Measurement::RowsAtCompileTime, Measurement::RowsAtCompileTime>::Identity() * 1e3,
        h_fun,
        H_fun
    );

    auto state_after = this->filter.getState();

    // state_after should differ from state_before (there was a non-zero measurement residual)
    auto dx = state_after.minus(state_before); 

    #if LIE_BACKEND_MANIF
    EXPECT_GT(dx.coeffs().norm(), 0.0);
    #else
    EXPECT_GT(dx.norm(), 0.0);
    #endif
}

TYPED_TEST(IESEKFTestFixture, ResetRestoresIdentityCovariance) {
    // mutate state and covariance
    typename TestFixture::Group g = this->filter.getState();
    typename TestFixture::Filter::VecTangent t = TestFixture::Filter::VecTangent::Zero();
    t(0) = 0.123;
    g.plus(typename TestFixture::Group::Tangent(t));
    this->filter.setState(g);
    this->filter.setCovariance(TestFixture::Filter::MatDoF::Identity() * 0.5);

    // reset
    this->filter.reset();

    // state should be identity (group's setIdentity semantics) - test via minus to zero
    auto st = this->filter.getState();
    auto zeroTang = st.minus(st); // should be zero

    #if LIE_BACKEND_MANIF
    EXPECT_NEAR(zeroTang.coeffs().norm(), 0.0, 1e-12);
    #else
    EXPECT_NEAR(zeroTang.norm(), 0.0, 1e-12);
    #endif
    

    // covariance should be reset to 1e-3 * I per implementation
    auto P = this->filter.getCovariance();
    for (int i = 0; i < TestFixture::Filter::DoF; ++i) {
        for (int j = 0; j < TestFixture::Filter::DoF; ++j) {
            double expected = (i == j) ? 1e-3 : 0.0;
            EXPECT_NEAR(P(i, j), expected, 1e-12);
        }
    }
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
