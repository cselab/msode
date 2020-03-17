#include "helpers.h"

#include <msode/analytic_control/helpers.h>
#include <msode/analytic_control/optimal_path.h>

#include <cstdio>
#include <gtest/gtest.h>
#include <vector>

using namespace msode;

constexpr real magneticFieldMagnitude {1.0_r};

static std::vector<RigidBody> generateRandomBodies(int n, std::mt19937& gen)
{
    std::vector<RigidBody> bodies;
    bodies.reserve(n);

    for (int i = 0; i < n; ++i)
        bodies.push_back(helpers::generateRandomBody(gen));

    return bodies;
}

static std::vector<real3> generateA(int n, std::mt19937& gen)
{
    const auto bodies = generateRandomBodies(n, gen);
    const auto V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);
    const auto U = V.inverse();

    const real3 boxLo {-50.0_r, -50.0_r, -50.0_r};
    const real3 boxHi {+50.0_r, +50.0_r, +50.0_r};
    const auto positions = analytic_control::generateRandomPositionsBox(n, boxLo, boxHi);

    return analytic_control::computeA(U, positions);
}

static inline Quaternion quaternionFromAngles(real theta, real phi, real psi)
{
    const real3 normal {std::cos(theta) * std::sin(phi),
                        std::sin(theta) * std::sin(phi),
                        std::cos(phi)};

    const auto q = Quaternion::createFromRotation(psi, normal);
    return q.normalized();
}

GTEST_TEST( AC_OPT, derivative )
{
    const int n = 4;
    std::mt19937 gen {4217};
    const auto A = generateA(n, gen);

    auto F = [&](real theta, real phi, real psi)
    {
        return analytic_control::computeTime(A, quaternionFromAngles(theta, phi, psi));
    };

    // TODO random tries
    const real theta {0.34545_r};
    const real phi {0.14545_r};
    const real psi {0.64545_r};

    const real h = 1e-3_r;
    
    // derivative wrt theta
    const real dFdtheta_FD = (F(theta + h, phi, psi) - F(theta, phi, psi)) / h;

    // TODO
    printf("%g\n", dFdtheta_FD);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
