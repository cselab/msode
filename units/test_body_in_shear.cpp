#include "helpers.h"

#include <msode/core/simulation.h>
#include <msode/core/velocity_field/shear.h>

#include <gtest/gtest.h>
#include <random>

using namespace msode;

// zero external field, so only the shear is driving the simulation
constexpr real magneticFieldMagnitude {0.0_r};


GTEST_TEST( SHEAR, sphere )
{
    std::mt19937 gen{424242L};
    const real G {5.0_r};

    std::vector<RigidBody> bodies = {helpers::generateRandomBody(gen)};

    auto velField = std::make_unique<VelocityFieldShear>(G);

    MagneticField magneticField(magneticFieldMagnitude,
                                [](real){return 0.0_r;},
                                [](real){return real3 {1.0_r, 0.0_r, 0.0_r};});

    Simulation sim(bodies, magneticField, std::move(velField));

    const real tEnd {10.0_r};
    const int nsteps = 100;
    const real dt {tEnd/nsteps};

    sim.runForwardEuler(nsteps, dt);

    const auto r = sim.getBodies()[0].r;
    const auto w = sim.getBodies()[0].omega;

    constexpr real eps {1e-5_r};

    ASSERT_NEAR(r.x, tEnd * r.y * G, eps);

    ASSERT_NEAR(w.x, 0.0_r, eps);
    ASSERT_NEAR(w.y, 0.0_r, eps);
    ASSERT_NEAR(w.z, 0.5_r * G, eps);
}


GTEST_TEST( SHEAR, ellipsoid )
{
    std::mt19937 gen{424242L};
    const real G {5.0_r};
    const real lambda = 2.0_r;

    std::vector<RigidBody> bodies = {helpers::generateRandomBody(gen)};
    bodies[0].aspectRatio = lambda;

    auto velField = std::make_unique<VelocityFieldShear>(G);

    MagneticField magneticField(magneticFieldMagnitude,
                                [](real){return 0.0_r;},
                                [](real){return real3 {1.0_r, 0.0_r, 0.0_r};});

    Simulation sim(bodies, magneticField, std::move(velField));

    const real tEnd {0.2_r};
    const int nsteps = 50;
    const real dt {tEnd/nsteps};

    real theta = M_PI/2;

    for (int i = 0; i < nsteps; ++i)
    {
        sim.runForwardEuler(100, dt/100);
        for (int j = 0; j < 100; ++j)
        {
            const real ct = std::cos(theta);
            const real st = std::sin(theta);
            const real omega = G * (ct*ct + lambda*lambda * st*st) / (1.0_r + lambda*lambda);
            theta += dt/100 * omega;
        }


        const auto r = sim.getBodies()[0].r;
        const auto w = sim.getBodies()[0].omega;

        constexpr real eps {1e-5_r};

        ASSERT_NEAR(r.x, tEnd * r.y * G, eps);

        const real ct = std::cos(theta);
        const real st = std::sin(theta);
        const real omega = G * (ct*ct + lambda*lambda * st*st) / (1.0_r + lambda*lambda);

        ASSERT_NEAR(w.x, 0.0_r, eps);
        ASSERT_NEAR(w.y, 0.0_r, eps);
        ASSERT_NEAR(w.z, omega, 1e-3_r);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
