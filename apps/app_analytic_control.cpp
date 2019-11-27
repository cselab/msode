#include "analytic_control/optimal_path.h"
#include "analytic_control/apply_strategy.h"

#include <iostream>

int main(int argc, char **argv)
{
    using namespace msode;
    
    if (argc < 2                     ||
        std::string(argv[1]) == "-h" ||
        std::string(argv[1]) == "--help")
    {
        fprintf(stderr, "usage : %s <swimmer0.cfg> <swimmer1.cfg>... \n\n", argv[0]);
        return 1;
    }

    const real magneticFieldMagnitude = 1.0_r;

    const real3 boxLo{-50.0_r, -50.0_r, -50.0_r};
    const real3 boxHi{+50.0_r, +50.0_r, +50.0_r};
    
    std::vector<RigidBody> bodies;
    for (int i = 1; i < argc; ++i)
    {
        const auto body = Factory::readRigidBodyConfig(argv[i]);
        bodies.push_back(body);
    }

    const MatrixReal V = createVelocityMatrix(magneticFieldMagnitude, bodies);
    const MatrixReal U = V.inverse();

    // std::cout << V << "\n\n";
    // std::cout << U << std::endl;

    const long seed = 42424242;
    auto initialPositions = generateRandomPositions(bodies.size(), boxLo, boxHi, seed);
    const real tTot = simulateOptimalPath(magneticFieldMagnitude, bodies, initialPositions, U, "ac_trajectories.txt", 1000);

    std::cout << "Took " << tTot << " time units to bring to target" << std::endl;
    
    return 0;
}
