#include <msode/core/simulation.h>
#include <msode/core/velocity_field/constant.h>

#include <gtest/gtest.h>
#include <random>

using namespace msode;

// zero external field, so only the advection is driving the simulation
constexpr real magneticFieldMagnitude {0.0_r};

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

GTEST_TEST( ADVECTION, constant )
{
    std::mt19937 gen{424242L};
    const real3 vel {10.0_r, -1.0_r, 5.0_r};
    
    std::vector<RigidBody> bodies = {generateRandomBody(gen)};

    auto velField = std::make_unique<VelocityFieldConstant>(vel);

    MagneticField magneticField(magneticFieldMagnitude,
                                [](real){return 0.0_r;},
                                [](real){return real3 {1.0_r, 0.0_r, 0.0_r};});

    Simulation sim(bodies, magneticField, std::move(velField));

    const real tEnd {10.0_r};
    const int nsteps = 100;
    const real dt {tEnd/nsteps};

    sim.runForwardEuler(nsteps, dt);

    const auto rEnd = sim.getBodies()[0].r;

    constexpr real eps {1e-5_r};
    
    ASSERT_NEAR(rEnd.x, tEnd * vel.x, eps);
    ASSERT_NEAR(rEnd.y, tEnd * vel.y, eps);
    ASSERT_NEAR(rEnd.z, tEnd * vel.z, eps);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
