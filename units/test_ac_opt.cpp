#include "helpers.h"
#include "timer.h"

#include <msode/analytic_control/helpers.h>
#include <msode/analytic_control/optimal_path.h>
#include <msode/core/factory.h>

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
        return analytic_control::computeTravelTime(A, quaternionFromAngles(theta, phi, psi));
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

        const real3 gradient = analytic_control::computeTravelTimeGradient(A, theta, phi, psi);

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
        qCMAES = analytic_control::findBestPathCMAES(A);
    const double tCMAES = timer.elapsedAndReset() / numTries;

    timer.start();
    for (int i = 0; i < numTries; ++i)
        qLBFGS = analytic_control::findBestPathLBFGS(A);
    const double tLBFGS = timer.elapsedAndReset() / numTries;

    const real ttCMAES = analytic_control::computeTravelTime(A, qCMAES);
    const real ttLBFGS = analytic_control::computeTravelTime(A, qLBFGS);
    
    printf("CMAES: %g in %g ms\n"
           "LBFGS: %g in %g ms\n",
           ttCMAES, tCMAES, ttLBFGS, tLBFGS);

    // check that the approximate optimizer is within 1% bounds
    ASSERT_NEAR(ttCMAES, ttLBFGS, 0.01_r * std::abs(ttCMAES));
}


// case that caused troubles in previous cmaes version
static std::vector<real3> computeADifficultCase()
{
    const Config config = json::parse(R"(
{
"bodies" : [
{
    "moment": [0.0, 10.0, 0.0],
    "quaternion": [0.0, 1.0, 0.0, 0.0],
    "position": [0.0, 0.0, 0.0],
    "propulsion": {
        "A": [0.2463907364779919, 0.20047533416665167, 0.19963144341455386],
        "B": [0.1, 0.0, 0.0],
        "C": [6.9836362589703, 1.1539878672490023, 1.2181219967949395]
    }
},
{
    "moment": [0.0, 3.0356727390869245, 0.0],
    "quaternion": [0.0, 1.0, 0.0, 0.0],
    "position": [0.0, 0.0, 0.0],
    "propulsion": {
        "A": [0.3239313882752511, 0.2413293038414949, 0.2543042668817562],
        "B": [0.32941627307981225, 0.0, 0.0],
        "C": [9.43265794343002, 1.681798137390105, 1.7429895150200194]
    }
}
]
})");

    std::vector<RigidBody> bodies;
    for (auto c : config.at("bodies"))
        bodies.push_back(factory::readRigidBodyFromConfig(c));

    const std::vector<real3> positions {real3{-6.82644, 5.90076, 1.85192},
                                        real3{0.976324, 3.19717, 3.25685}};

    const auto V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);
    const auto U = V.inverse();
    return analytic_control::computeA(U, positions);
}

// GTEST_TEST( AC_OPT, korali_comparison )
// {
//     const auto A = computeADifficultCase();

//     const bool verbose = false;
    
//     for (long seed = 4242425; seed < 4242435; ++seed)
//     {
//         printf("\nseed %ld:\n", seed);
//         auto qKorali = analytic_control::findBestPathCMAESKorali(A, seed, verbose);
//         const real ttKorali = analytic_control::computeTravelTime(A, qKorali);
//         printf("korali: %g\n", ttKorali);
           
//         auto qCMAES = analytic_control::findBestPathCMAES(A, seed, verbose);
//         const real ttCMAES = analytic_control::computeTravelTime(A, qCMAES);
//         printf("CMAES:  %g\n", ttCMAES);
//     }
// }


GTEST_TEST( AC_OPT, robustness )
{
    const auto A = computeADifficultCase();

    const bool verbose = false;
    const int numTries = 1000;
    std::mt19937 gen {42424242};

    std::vector<real> results;
    results.reserve(numTries);
    
    for (int i = 0; i < numTries; ++i)
    {
        const long seed = gen();
        auto q = analytic_control::findBestPathCMAES(A, seed, verbose);
        const real tt = analytic_control::computeTravelTime(A, q);
        results.push_back(tt);
    }

    const real best = *std::min_element(results.begin(), results.end());
    const real tol = 1e-3;
    int numFailed = 0;
    for (auto v : results)
    {
        if (v > best + tol)
            ++numFailed;
    }

    printf("%d failed (%g%)\n", numFailed, 100.0 * (double) numFailed / (double) numTries);
}

// GTEST_TEST( AC_OPT, robustnessKorali )
// {
//     const auto A = computeADifficultCase();

//     const bool verbose = false;
//     const int numTries = 1000;
//     std::mt19937 gen {42424242};

//     std::vector<real> results;
//     results.reserve(numTries);
    
//     for (int i = 0; i < numTries; ++i)
//     {
//         const long seed = gen();
//         auto q = analytic_control::findBestPathCMAESKorali(A, seed, verbose);
//         const real tt = analytic_control::computeTravelTime(A, q);
//         results.push_back(tt);
//     }

//     const real best = *std::min_element(results.begin(), results.end());
//     const real tol = 1e-3;
//     int numFailed = 0;
//     for (auto v : results)
//     {
//         if (v > best + tol)
//             ++numFailed;
//     }

//     printf("%d failed (%g%)\n", numFailed, 100.0 * (double) numFailed / (double) numTries);
// }


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
