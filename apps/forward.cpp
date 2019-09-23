#include "simulation.h"
#include "factory.h"

#include <iostream>

constexpr real magneticFieldMagnitude {1.0_r};

static void runAndDump(RigidBody body, real omega, const std::string& out, int dumpEvery)
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

    simulation.activateDump(out, dumpEvery);
    simulation.run(nsteps, dt);
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        fprintf(stderr, "usage : %s <swimmer.cfg> <omega> <fps> <trajectory_file> \n\n", argv[0]);
        return 1;
    }

    const auto body    = Factory::readRigidBodyConfig(argv[1]);
    const real omega   = static_cast<real>(std::stod(argv[2]));
    const int fps      = std::stoi(argv[3]);
    const std::string out (argv[4]);

    const int dumpEvery = fps;
    
    runAndDump(body, omega, out, dumpEvery);
    
    return 0;
}
