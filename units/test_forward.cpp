#include <msode/core/simulation.h>
#include <msode/utils/mean_vel.h>

#include <gtest/gtest.h>
#include <random>

using namespace msode;

constexpr real magneticFieldMagnitude {3.5_r};

static PropulsionMatrix generateRandomPropulsion(std::mt19937& gen)
{
    std::uniform_real_distribution<real> unif(0.8, 1.2);
    PropulsionMatrix P;
    P.A[0]          = 0.3_r * unif(gen);
    P.A[1] = P.A[2] = 0.2_r * unif(gen);

    P.B[0] = 0.2_r;
    P.B[1] = P.B[2] = 0.0_r;

    P.C[0]          = 6.3_r * unif(gen);
    P.C[1] = P.C[2] = 1.2_r * unif(gen);
    return P;
}

static RigidBody generateRandomBody(std::mt19937& gen)
{
    const auto q = Quaternion::createIdentity();
    const real3 r {0.0_r, 0.0_r, 0.0_r};
    const real3 m {0.0_r, 2.0_r, 0.0_r};
    const auto propulsion = generateRandomPropulsion(gen);
    
    return {q, r, m, propulsion};    
}

static void compareODEvsIntegral(real coeffLo, real coeffHi, long numTests = 10)
{
    std::mt19937 gen(42);
    const auto body = generateRandomBody(gen);
    const real omegaC = body.stepOutFrequency(magneticFieldMagnitude);

    constexpr real tEndODE = 500.0_r;
    constexpr long nIntegrationSteps = 1000;

    std::uniform_real_distribution<real> omegaDistr(coeffLo, coeffHi * omegaC);

    for (long i = 0; i < numTests; ++i)
    {
        const real omega = omegaDistr(gen);

        const real vODE = computeMeanVelocityODE       (body, magneticFieldMagnitude, omega, tEndODE);
        const real vInt = computeMeanVelocityAnalytical(body, magneticFieldMagnitude, omega, nIntegrationSteps);

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
