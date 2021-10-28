#include "helpers.h"

#include <msode/core/simulation.h>
#include <msode/core/velocity_field/none.h>

#include <gtest/gtest.h>
#include <random>

using namespace msode;

// zero external field, so only thermal noise is driving the simulation
constexpr real magneticFieldMagnitude {0.0_r};


GTEST_TEST( THERMAL_NOISE, translation )
{
    const real D {1.2345_r};

    std::mt19937 gen{424242L};
    std::vector<RigidBody> bodies = {helpers::generateRandomBody(gen)};
    bodies[0].transDiffusion = D;

    auto velField = std::make_unique<VelocityFieldNone>();
    MagneticField magneticField(magneticFieldMagnitude,
                                [](real){return 0.0_r;},
                                [](real){return real3 {1.0_r, 0.0_r, 0.0_r};});

    Simulation sim(bodies, magneticField, std::move(velField));

    const int nSamples = 5000;

    const real tEnd {10.0_r};
    const int nsteps = 100;
    const real dt {tEnd/nsteps};

    real MSD = 0.0_r;

    for (int sample = 0; sample < nSamples; ++sample)
    {
        const real3 x0 = sim.getBodies()[0].r;
        sim.runForwardEuler(nsteps, dt);
        const real3 x1 = sim.getBodies()[0].r;
        const real3 dx = x1 - x0;
        MSD += dot(dx, dx);
    }
    MSD /= nSamples;

    const real DMSD = MSD / (tEnd * 6.0_r);
    ASSERT_NEAR(DMSD, D, 1e-1_r);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
