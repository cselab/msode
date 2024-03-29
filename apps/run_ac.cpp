// Copyright 2020 ETH Zurich. All Rights Reserved.
/** run_ac

    Finds a sequence of magnetic field rotation frequencies and directions to bring given ABFs to a target position optimally.
 */


#include <msode/analytic_control/optimal_path.h>
#include <msode/analytic_control/apply_strategy.h>
#include <msode/core/velocity_field/factory.h>

#include <iostream>

using namespace msode;

static std::vector<RigidBody> readBodies(const Config& config)
{
    if (!config.is_array())
        msode_die("Expected an array of bodies in config");

    std::vector<RigidBody> bodies;

    for (const auto& c : config)
        bodies.push_back(msode::factory::readRigidBodyFromConfig(c));

    return bodies;
}

int main(int argc, char **argv)
{
    if (argc != 3                    ||
        std::string(argv[1]) == "-h" ||
        std::string(argv[1]) == "--help")
    {
        fprintf(stderr, "usage : %s <config.json> <L>\n\n", argv[0]);
        return 1;
    }

    std::ifstream confFile(argv[1]);
    const real L = static_cast<real>(std::atof(argv[2]));

    if (!confFile.is_open())
        msode_die("Could not open the config file '%s'", argv[1]);

    const Config config = json::parse(confFile);

    const real magneticFieldMagnitude = config.at("fieldMagnitude");
    const auto bodies = readBodies(config.at("bodies"));
    auto velocityField = factory::createVelocityField(config, ConfPointer("/velocityField"));

    const real3 boxLo{-L, -L, -L};
    const real3 boxHi{+L, +L, +L};

    const analytic_control::MatrixReal V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);
    const analytic_control::MatrixReal U = V.inverse();

    const long seed = 42424242;
    const int dumpEvery = config.at("dumpEvery");
    auto initialPositions = analytic_control::generateRandomPositionsBox(bodies.size(), boxLo, boxHi, seed);
    const bool verbose = true;

    const real tTot = analytic_control::simulateOptimalPath(magneticFieldMagnitude, bodies, initialPositions,
                                                            std::move(velocityField), U, "ac_trajectories.dat",
                                                            dumpEvery, verbose);

    std::cout << "Took " << tTot << " time units to bring to target" << std::endl;

    return 0;
}
