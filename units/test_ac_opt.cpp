#include "helpers.h"
#include "timer.h"


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
    const real3 normal {std::cos(phi) * std::sin(psi),
                        std::sin(phi) * std::sin(psi),
                        std::cos(psi)};

    const auto q = Quaternion::createFromRotation(theta, normal);
    return q.normalized();
}

GTEST_TEST( AC_OPT, derivative )
{
    const int n = 4;
    const int numTries = 5;
    std::mt19937 gen {4217};
    const auto A = generateA(n, gen);

    auto F = [&](real theta, real phi, real psi)
    {
        return analytic_control::computeTime(A, quaternionFromAngles(theta, phi, psi));
    };

    std::uniform_real_distribution<real> dstr(0.0_r, 1.0_r);
    
    for (int i = 0; i < numTries; ++i)
    {
        const real theta = dstr(gen) * 2 * M_PI;
        const real phi   = dstr(gen) * 2 * M_PI;
        const real psi   = dstr(gen) * 1 * M_PI;

        const real h = 1e-4_r;
        const real tolerance = 1e-3_r;

        const real3 gradient_FD = {(F(theta + h, phi, psi) - F(theta - h, phi, psi)) / (2*h),
                                   (F(theta, phi + h, psi) - F(theta, phi - h, psi)) / (2*h),
                                   (F(theta, phi, psi + h) - F(theta, phi, psi - h)) / (2*h)};

        const real3 gradient = analytic_control::computeTimeGradient(A, theta, phi, psi);

        ASSERT_NEAR(gradient_FD.x, gradient.x, tolerance);
        ASSERT_NEAR(gradient_FD.y, gradient.y, tolerance);
        ASSERT_NEAR(gradient_FD.z, gradient.z, tolerance);

        // printf("%g  %g\n", gradient_FD.x, gradient.x);
        // printf("%g  %g\n", gradient_FD.y, gradient.y);
        // printf("%g  %g\n\n", gradient_FD.z, gradient.z);
    }
}

GTEST_TEST( AC_OPT, optimum )
{
    const int n = 4;
    std::mt19937 gen {4217};
    const auto A = generateA(n, gen);

    mTimer timer;
    Quaternion qCMAES, qLBFGS;

    const int numTries {10};

    timer.start();
    for (int i = 0; i < numTries; ++i)
        qCMAES = analytic_control::findBestPath(A);
    const double tCMAES = timer.elapsedAndReset() / numTries;

    timer.start();
    for (int i = 0; i < numTries; ++i)
        qLBFGS = analytic_control::findBestPathLBFGS(A);
    const double tLBFGS = timer.elapsedAndReset() / numTries;

    const real ttCMAES = analytic_control::computeTime(A, qCMAES);
    const real ttLBFGS = analytic_control::computeTime(A, qLBFGS);
    
    printf("CMAES: %g in %g ms\n"
           "LBFGS: %g in %g ms\n",
           ttCMAES, tCMAES, ttLBFGS, tLBFGS);

    // check that the approximate optimizer is within 1% bounds
    ASSERT_NEAR(ttCMAES, ttCMAES, 0.01_r * std::abs(ttCMAES));
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
