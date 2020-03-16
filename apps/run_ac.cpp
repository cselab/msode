#include <msode/analytic_control/optimal_path.h>
#include <msode/analytic_control/apply_strategy.h>

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
    if (argc != 2                    ||
        std::string(argv[1]) == "-h" ||
        std::string(argv[1]) == "--help")
    {
        fprintf(stderr, "usage : %s <config.json>\n\n", argv[0]);
        return 1;
    }

    std::ifstream confFile(argv[1]);

    if (!confFile.is_open())
        msode_die("Could not open the config file '%s'", argv[1]);

    const Config config = json::parse(confFile);

    const real magneticFieldMagnitude = config.at("fieldMagnitude");
    const auto bodies = readBodies(config.at("bodies"));

    const real3 boxLo{-50.0_r, -50.0_r, -50.0_r};
    const real3 boxHi{+50.0_r, +50.0_r, +50.0_r};

    const analytic_control::MatrixReal V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);
    const analytic_control::MatrixReal U = V.inverse();

    // std::cout << V << "\n\n";
    // std::cout << U << std::endl;

    const long seed = 42424242;
    auto initialPositions = analytic_control::generateRandomPositionsBox(bodies.size(), boxLo, boxHi, seed);
    const real tTot = analytic_control::simulateOptimalPath(magneticFieldMagnitude, bodies, initialPositions, U, "ac_trajectories.dat", 1000);

    std::cout << "Took " << tTot << " time units to bring to target" << std::endl;
    
    return 0;
}
