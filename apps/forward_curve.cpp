#include "simulation.h"
#include "factory.h"

#include <iostream>

constexpr real magneticFieldMagnitude {1.0_r};

static real stepOutFrequency(const RigidBody body)
{
    return body.propulsion.C[0] * magneticFieldMagnitude * length(body.magnMoment);
}

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
    if (argc != 2)
    {
        fprintf(stderr, "usage : %s swimmer.cfg\n\n", argv[0]);
        return 1;
    }

    const auto body = Factory::readRigidBodyConfig(argv[1]);

    const real omegaC = stepOutFrequency(body);
    const int nOmegas = 100;
    const real maxOmega = 2.0_r * omegaC;
    const real dOmega = maxOmega / nOmegas;
    
    for (int i = 0; i < nOmegas; ++i)
    {
        const real omega = i * dOmega;
        const real v = computeMeanVelocity(body, omega);
        std::cout << omega << " " << v << std::endl;
    }
    
    return 0;
}
