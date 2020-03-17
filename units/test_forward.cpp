#include "helpers.h"

#include <msode/core/simulation.h>
#include <msode/utils/mean_vel.h>

#include <gtest/gtest.h>
#include <random>

using namespace msode;

constexpr real magneticFieldMagnitude {3.5_r};

static void compareODEvsIntegral(real coeffLo, real coeffHi, long numTests = 10)
{
    std::mt19937 gen(42);
    const auto body = helpers::generateRandomBody(gen);
    const real omegaC = body.stepOutFrequency(magneticFieldMagnitude);

    constexpr real tEndODE = 500.0_r;
    constexpr long nIntegrationSteps = 1000;

    std::uniform_real_distribution<real> omegaDistr(coeffLo, coeffHi * omegaC);

    for (long i = 0; i < numTests; ++i)
    {
        const real omega = omegaDistr(gen);

        const real vODE = utils::computeMeanVelocityODE       (body, magneticFieldMagnitude, omega, tEndODE);
        const real vInt = utils::computeMeanVelocityAnalytical(body, magneticFieldMagnitude, omega, nIntegrationSteps);

        ASSERT_NEAR(vODE, vInt, 0.01_r);
    }
}

GTEST_TEST( MEAN_VELOCITY, ODE_vs_integral_linear )
{
    compareODEvsIntegral(0.5_r, 0.9_r);
}

GTEST_TEST( MEAN_VELOCITY, ODE_vs_integral_non_linear )
{
    compareODEvsIntegral(1.2_r, 3.0_r);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
