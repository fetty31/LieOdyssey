#include <gtest/gtest.h>
#include <backends/liepp_groups.hpp>

using namespace lie_odyssey;

// ---------------------- SO3LiePP Tests ----------------------

TEST(SO3LiePPTest, ExpLogConsistency) {
    Eigen::Matrix<double, 3, 1> xi;
    xi.setRandom();

    auto so3 = SO3LiePP<>::Exp(xi);
    auto xi_back = so3.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], xi_back[i], 1e-9);
    }
}

TEST(SO3LiePPTest, MultiplicationInverse) {
    SO3LiePP<> a = SO3LiePP<>::Exp(Eigen::Matrix<double, 3, 1>::Random());
    auto inv_a = a.Inverse();

    auto identity = (a * inv_a).asMatrix();
    Eigen::Matrix3d I = Eigen::Matrix3d::Identity();

    EXPECT_TRUE((identity - I).norm() < 1e-9);
}

// ---------------------- SE3LiePP Tests ----------------------

TEST(SE3LiePPTest, ExpLogConsistency) {
    Eigen::Matrix<double, 6, 1> xi;
    xi.setRandom();

    auto se3 = SE3LiePP<>::Exp(xi);
    auto xi_back = se3.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], xi_back[i], 1e-9);
    }
}

TEST(SE3LiePPTest, MultiplicationInverse) {
    SE3LiePP<> a = SE3LiePP<>::Exp(Eigen::Matrix<double, 6, 1>::Random());
    auto inv_a = a.Inverse();

    auto identity = (a * inv_a).asMatrix();
    Eigen::Matrix4d I = Eigen::Matrix4d::Identity();

    EXPECT_TRUE((identity - I).norm() < 1e-9);
}

// ---------------------- SE23LiePP Tests ----------------------

TEST(SE23LiePPTest, ExpLogConsistency) {
    Eigen::Matrix<double, 9, 1> xi;
    xi.setRandom();

    auto se23 = SE23LiePP<>::Exp(xi);
    auto xi_back = se23.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], xi_back[i], 1e-9);
    }
}

TEST(SE23LiePPTest, MultiplicationInverse) {
    SE23LiePP<> a = SE23LiePP<>::Exp(Eigen::Matrix<double, 9, 1>::Random());
    auto inv_a = a.Inverse();

    auto identity = (a * inv_a).asMatrix();
    EXPECT_TRUE((identity - identity.Identity()).norm() < 1e-9);
}

// ---------------------- SEn3LiePP (N=3) Tests ----------------------

TEST(SEn3LiePPTest, ExpLogConsistency) {
    Eigen::Matrix<double, 12, 1> xi; // For N=3 -> 12 DoF
    xi.setRandom();

    auto sen3 = SEn3LiePP<double, 3>::Exp(xi);
    auto xi_back = sen3.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], xi_back[i], 1e-9);
    }
}

TEST(SEn3LiePPTest, MultiplicationInverse) {
    auto a = SEn3LiePP<double, 3>::Exp(Eigen::Matrix<double, 12, 1>::Random());
    auto inv_a = a.Inverse();

    auto identity = (a * inv_a).asMatrix();
    EXPECT_TRUE(identity.isApprox(identity.Identity(), 1e-9));
}

// ---------------------- Gal3LiePP Tests ----------------------

TEST(Gal3LiePPTest, ExpLogConsistency) {
    Eigen::Matrix<double, 10, 1> xi; // DoF of Gal(3) is 10
    xi.setRandom();

    auto gal3 = Gal3LiePP<>::Exp(xi);
    auto xi_back = gal3.Log();

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], xi_back[i], 1e-9);
    }
}

TEST(Gal3LiePPTest, MultiplicationInverse) {
    auto a = Gal3LiePP<>::Exp(Eigen::Matrix<double, 10, 1>::Random());
    auto inv_a = a.Inverse();

    auto identity = (a * inv_a).asMatrix();
    EXPECT_TRUE(identity.isApprox(identity.Identity(), 1e-9));
}

// ---------------------- Gal3TGLiePP Tests ----------------------

// TEST(Gal3TGLiePPTest, ExpLogConsistency) {
//     Eigen::Matrix<double, 16, 1> xi; // DoF of Gal3TG is 16
//     xi.setRandom();

//     auto gal3tg = Gal3TGLiePP<>::Exp(xi);
//     auto xi_back = gal3tg.Log();

//     for (int i = 0; i < xi.size(); ++i) {
//         EXPECT_NEAR(xi[i], xi_back[i], 1e-9);
//     }
// }

// TEST(Gal3TGLiePPTest, MultiplicationInverse) {
//     auto a = Gal3TGLiePP<>::Exp(Eigen::Matrix<double, 16, 1>::Random());
//     auto inv_a = a.Inverse();

//     auto identity = (a * inv_a).asMatrix();
//     EXPECT_TRUE(identity.isApprox(identity.Identity(), 1e-9));
// }

// ---------------------- Main ----------------------

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
