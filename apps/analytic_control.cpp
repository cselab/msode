#include "ac/helpers.h"
#include "ac/optimal_path.h"

#include <iostream>

int main(int argc, char **argv)
{
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
        const RigidBody body = Factory::readRigidBodyConfig(argv[i]);
        bodies.push_back(body);
    }

    const MatrixReal V = createVelocityMatrix(magneticFieldMagnitude, bodies);
    const MatrixReal U = V.inverse();

    // std::cout << V << "\n\n";
    // std::cout << U << std::endl;

    for (int i = 0; i < 1; ++i)
    {
        std::cout << "=========== step " << i << std::endl;
        std::vector<real3> initialPositions = generateRandomPositions(bodies.size(), boxLo, boxHi, 42 * i + 13);
        auto A = computeA(U, initialPositions);
        
        auto n = findBestPlane(A);
        std::cout << n << "\t\t" << computeTime(n) << std::endl;
    }
    return 0;
}
