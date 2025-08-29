#include <iostream>

#include <groups/SEn3.hpp>

int main(int argc, char **argv)
{
    using SO3d = group::SO3<double>;
    using SE23d = group::SEn3<double, 2>;
    using Vector3d = Eigen::Matrix<double, 3, 1>;
    using Vector9d = Eigen::Matrix<double, 9, 1>;
    using Matrix9d = Eigen::Matrix<double, 9, 9>;

    // Define random extended pose X via exponential map
    Vector9d x = Vector9d::Random();
    SE23d X = SE23d::exp(x);

    // Define Extended pose Y with identity rotation, and given velocity and position
    Vector3d p = Vector3d(1, 2, 3);
    Vector3d v = Vector3d(0.1, 0.1, 0.3);
    SO3d R = SO3d();
    SE23d Y = SE23d(R, {v, p});

    // Get extended pose composition Z = XY
    SE23d Z = X*Y;

    // print Z as matrix
    std::cout << Z.asMatrix() << std::endl;

    // print Rotational component of Z as quaternion, position and velocity
    std::cout << Z.q() << '\n' << Z.p() << '\n'<< Z.v() << '\n' << std::endl;

    // get Adjoint matrix of SE23
    Matrix9d AdZ = Z.Adjoint();

    return 0;
}