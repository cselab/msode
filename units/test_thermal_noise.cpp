#include <msode/core/simulation.h>
#include <msode/core/velocity_field/none.h>

#include <gtest/gtest.h>
#include <random>

using namespace msode;

// zero external field, so only thermal noise is driving the simulation
constexpr real magneticFieldMagnitude {0.0_r};

RigidBody createSphere(real eta, real a)
{
    PropulsionMatrix p;
    p.A[0] = p.A[1] = p.A[2] = 1.0_r / (6 * M_PI * eta * a);
    p.B[0] = p.B[1] = p.B[2] = 0.0_r;
    p.C[0] = p.C[1] = p.C[2] = 1.0_r / (8 * M_PI * eta * a*a*a);

    RigidBody b{Quaternion::createIdentity(), // q
                make_real3(0.0_r), make_real3(0.0_r), // r, m
                p};
    return b;
}


GTEST_TEST( THERMAL_NOISE, translation )
{
    const real kBT{2.0_r};
    const real a{1.0_r};
    const real eta{1.0_r};

    std::vector<RigidBody> bodies = {createSphere(eta, a)};

    auto velField = std::make_unique<VelocityFieldNone>();
    MagneticField magneticField(magneticFieldMagnitude,
                                [](real){return 0.0_r;},
                                [](real){return real3 {1.0_r, 0.0_r, 0.0_r};});

    Simulation sim(bodies, magneticField, kBT, std::move(velField));

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
    const real D = kBT / (6 * M_PI * eta * a); // Einstein-stokes relation
    ASSERT_NEAR(DMSD, D, D * 1e-2_r);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
