#include "simulation.h"
#include "factory.h"

#include <iostream>

static inline real meanVelocity(real3 r0, real3 r1, real T)
{
    return length(r0-r1)/T;
}

static real computeMeanVelocity(RigidBody body, real omega)
{
    constexpr real tEnd = 200.0_r;
    constexpr real dt {1e-3_r};
    constexpr long nsteps = tEnd / dt;

    constexpr real3 rStart {0.0_r, 0.0_r, 0.0_r};
    body.r = rStart;
        
    const real magneticFieldMagnitude {1.0_r};
    auto omegaField        = [omega](real t) {return omega;};
    auto rotatingDirection = []     (real t) {return real3{1.0_r, 0.0_r, 0.0_r};};

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};
    const std::vector<RigidBody> rigidBodies {body};
    Simulation simulation {rigidBodies, magneticField};

    simulation.run(nsteps, dt);

    const real3 rEnd = simulation.getBodies()[0].r;

    return meanVelocity(rStart, rEnd, tEnd);
}

int main(int argc, char **argv)
{
    Expect(argc == 2, "usage : ./main swimmer.cfg");

    const auto body = Factory::readRigidBodyConfig(argv[1]);

    const int nOmegas = 200;
    for (int i = 0; i < nOmegas; ++i)
    {
        const real omega = i * 0.1_r;
        const real v = computeMeanVelocity(body, omega);
        std::cout << omega << " " << v << std::endl;
    }
    
    return 0;
}
