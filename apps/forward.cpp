// Copyright 2020 ETH Zurich. All Rights Reserved.
/** forward

    ABF in a field rotating with frequency `omega` in the yz plane.
 */

#include <msode/core/simulation.h>
#include <msode/core/factory.h>

#include <iostream>

using namespace msode;

constexpr real magneticFieldMagnitude {1.0_r};
constexpr real dt {1e-3_r};

static void runAndDump(RigidBody body, real omega, const std::string& out, int dumpEvery)
{
    const int nRevolutions = 10;
    const real tEnd = nRevolutions * 2 * M_PI / omega;
    const long nsteps = tEnd / dt;

    constexpr real3 rStart {0.0_r, 0.0_r, 0.0_r};
    body.r = rStart;

    auto omegaField        = [omega](real) {return omega;};
    auto rotatingDirection = []     (real) {return real3{1.0_r, 0.0_r, 0.0_r};};

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};
    const std::vector<RigidBody> rigidBodies {body};
    Simulation simulation {rigidBodies, magneticField};

    simulation.activateDump(out, dumpEvery);
    simulation.runForwardEuler(nsteps, dt);
}

int main(int argc, char **argv)
{
    if (argc != 5)
    {
        fprintf(stderr, "usage : %s <swimmer.json> <omega> <fps> <trajectory_file> \n\n", argv[0]);
        return 1;
    }

    const auto body    = factory::readRigidBodyConfigFromFile(argv[1]);
    const real omega   = static_cast<real>(std::stod(argv[2]));
    const int fps      = std::stoi(argv[3]);
    const std::string out (argv[4]);

    const real tDumpEvery = 1.0_r / fps;
    const int dumpEvery = static_cast<int>(tDumpEvery / dt);

    runAndDump(body, omega, out, dumpEvery);

    return 0;
}
