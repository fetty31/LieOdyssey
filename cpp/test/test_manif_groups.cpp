#include <gtest/gtest.h>
#include <lie_odyssey/backends/manif_groups.hpp>  

using namespace lie_odyssey;

// ---------------------- SO3Manif Tests ----------------------

TEST(SO3ManifTest, ExpLogConsistency) {
    Eigen::Matrix<double, 3, 1> xi;
    xi.setRandom();

    auto so3_exp = SO3Manif<>::Exp(xi);
    auto so3_log = so3_exp.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], so3_log[i], 1e-9);
    }
}

TEST(SO3ManifTest, MultiplicationInverse) {
    auto a = SO3Manif<>::Exp(Eigen::Matrix<double, 3, 1>::Random());
    auto b = SO3Manif<>::Exp(Eigen::Matrix<double, 3, 1>::Random());

    auto ab = a * b;
    auto ab_inv = ab.Inverse();

    auto identity = (ab * ab_inv).asMatrix();
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity(); // Watch out! SO3Manif uses 4x4 matrix as MatrixType instead of 3x3
    EXPECT_NEAR((identity - I).norm(), 0.0, 1e-9);
}

// ---------------------- SE3Manif Tests ----------------------

TEST(SE3ManifTest, ExpLogConsistency) {
    Eigen::Matrix<double, 6, 1> xi;
    xi.setRandom();

    auto se3_exp = SE3Manif<>::Exp(xi);
    auto se3_log = se3_exp.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], se3_log[i], 1e-9);
    }
}

TEST(SE3ManifTest, MultiplicationInverse) {
    auto a = SE3Manif<>::Exp(Eigen::Matrix<double, 6, 1>::Random());
    auto b = SE3Manif<>::Exp(Eigen::Matrix<double, 6, 1>::Random());

    auto ab = a * b;
    auto ab_inv = ab.Inverse();

    auto identity = (ab * ab_inv).asMatrix();
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();
    EXPECT_NEAR((identity - I).norm(), 0.0, 1e-9);
}

// ---------------------- SE23Manif Tests ----------------------

TEST(SE23ManifTest, ExpLogConsistency) {
    Eigen::Matrix<double, 9, 1> xi;
    xi.setRandom();

    auto se23_exp = SE23Manif<>::Exp(xi);
    auto se23_log = se23_exp.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], se23_log[i], 1e-9);
    }
}

TEST(SE23ManifTest, MultiplicationInverse) {
    auto a = SE23Manif<>::Exp(Eigen::Matrix<double, 9, 1>::Random());
    auto b = SE23Manif<>::Exp(Eigen::Matrix<double, 9, 1>::Random());

    auto ab = a * b;
    auto ab_inv = ab.Inverse();

    auto identity = (ab * ab_inv).asMatrix();
    Eigen::Matrix<double,5,5> I = Eigen::Matrix<double,5,5>::Identity();
    EXPECT_NEAR((identity - I).norm(), 0.0, 1e-9);
}

// ---------------------- Gal3Manif Tests ----------------------

TEST(Gal3ManifTest, ExpLogConsistency) {
    Eigen::Matrix<double, 10, 1> xi;
    xi.setRandom();

    auto gal3_exp = Gal3Manif<>::Exp(xi);
    auto gal3_log = gal3_exp.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], gal3_log[i], 1e-9);
    }
}

TEST(Gal3ManifTest, MultiplicationInverse) {
    auto a = Gal3Manif<>::Exp(Eigen::Matrix<double, 10, 1>::Random());
    auto b = Gal3Manif<>::Exp(Eigen::Matrix<double, 10, 1>::Random());

    auto ab = a * b;
    auto ab_inv = ab.Inverse();

    auto identity = (ab * ab_inv).asMatrix();
    Eigen::Matrix<double,5,5> I = Eigen::Matrix<double,5,5>::Identity(); 
    EXPECT_NEAR((identity - I).norm(), 0.0, 1e-9);
}

// ---------------------- BundleManif Tests ----------------------

using PoseBiasBundle = BundleManif<double, manif::SE3, manif::R3>;

TEST(BundleManifTest, ExpLogConsistency) {
    Eigen::Matrix<double, PoseBiasBundle::DoF, 1> xi;
    xi.setRandom();

    auto bundle_exp = PoseBiasBundle::Exp(xi);
    auto bundle_log = bundle_exp.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], bundle_log[i], 1e-9);
    }
}

TEST(BundleManifTest, MultiplicationInverse) {
    auto a = PoseBiasBundle::Exp(Eigen::Matrix<double, PoseBiasBundle::DoF, 1>::Random());
    auto b = PoseBiasBundle::Exp(Eigen::Matrix<double, PoseBiasBundle::DoF, 1>::Random());

    auto ab = a * b;
    auto ab_inv = ab.Inverse();

    auto se3_group = (ab * ab_inv).subgroup<0>();
    auto identity = se3_group.quat().toRotationMatrix();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(identity.rows(), identity.cols());
    EXPECT_NEAR((identity - I).norm(), 0.0, 1e-9);
}

TEST(BundleManifTest, SubgroupAccess) {
    auto bundle = PoseBiasBundle::Identity();

    auto pose = bundle.subgroup<0>(); // SE3
    auto bias = bundle.subgroup<1>(); // R^3

    EXPECT_NEAR(pose.quat().toRotationMatrix()(0,0), 1.0, 1e-9);
    EXPECT_TRUE(bias.coeffs().isApprox(Eigen::Vector3d::Zero()));
}

// ---------------------- Main ----------------------

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
