#include <gtest/gtest.h>

#include <backends/liepp_groups.hpp>

using namespace lie_odyssey;

// ---------------------- Gal3LiePP Tests ----------------------

TEST(Gal3LiePPTest, ExpLogConsistency) {
    Gal3LiePP<> gal3;
    Eigen::Matrix<double, 6, 1> xi;
    xi.setRandom();

    auto gal3_exp = Gal3LiePP<>::Exp(xi);
    auto gal3_log = Gal3LiePP<>::Log(gal3_exp);

    // Check that Log(Exp(x)) ≈ x
    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], gal3_log[i], 1e-9);
    }
}

TEST(Gal3LiePPTest, MultiplicationInverse) {
    Gal3LiePP<> a, b;
    a.g = Gal3LiePP<>::Exp(Eigen::Matrix<double, 6, 1>::Random()).g;
    b.g = Gal3LiePP<>::Exp(Eigen::Matrix<double, 6, 1>::Random()).g;

    auto ab = a * b;
    auto ab_inv = ab.Inverse();

    // ab * ab_inv should be identity
    auto identity = ab.g * ab_inv.g;
    Eigen::Matrix<double, 6, 6> I = Eigen::Matrix<double, 6, 6>::Identity();
    EXPECT_TRUE((identity.asMatrix() - I).norm() < 1e-9);
}

// ---------------------- TGLiePP Tests ----------------------

TEST(TGLiePPTest, ExpLogConsistency) {
    Eigen::Matrix<double, 9, 1> xi;
    xi.setRandom();

    auto tg_exp = TGLiePP<>::Exp(xi);
    auto tg_log = TGLiePP<>::Log(tg_exp);

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], tg_log[i], 1e-9);
    }
}

TEST(TGLiePPTest, MultiplicationInverse) {
    TGLiePP<> a, b;
    a.g = TGLiePP<>::Exp(Eigen::Matrix<double, 9, 1>::Random()).g;
    b.g = TGLiePP<>::Exp(Eigen::Matrix<double, 9, 1>::Random()).g;

    auto ab = a * b;
    auto ab_inv = ab.Inverse();

    auto identity = ab.g * ab_inv.g;
    Eigen::Matrix<double, 9, 9> I = Eigen::Matrix<double, 9, 9>::Identity();
    EXPECT_TRUE((identity.asMatrix() - I).norm() < 1e-9);
}

// ---------------------- SDBLiePP Tests ----------------------

TEST(SDBLiePPTest, ExpLogConsistency) {
    Eigen::Matrix<double, 9, 1> xi;
    xi.setRandom();

    auto sdb_exp = SDBLiePP<>::Exp(xi);
    auto sdb_log = SDBLiePP<>::Log(sdb_exp);

    for (int i = 0; i < xi.size(); ++i) {
        EXPECT_NEAR(xi[i], sdb_log[i], 1e-9);
    }
}

TEST(SDBLiePPTest, MultiplicationInverse) {
    SDBLiePP<> a, b;
    a.g = SDBLiePP<>::Exp(Eigen::Matrix<double, 9, 1>::Random()).g;
    b.g = SDBLiePP<>::Exp(Eigen::Matrix<double, 9, 1>::Random()).g;

    auto ab = a * b;
    auto ab_inv = ab.Inverse();

    auto identity = ab.g * ab_inv.g;
    Eigen::Matrix<double, 9, 9> I = Eigen::Matrix<double, 9, 9>::Identity();
    EXPECT_TRUE((identity.asMatrix() - I).norm() < 1e-9);
}

// ---------------------- Main ----------------------

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
