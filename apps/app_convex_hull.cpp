#include <msode/core/simulation.h>
#include <msode/core/factory.h>
#include <msode/utils/mean_vel.h>

#include <iostream>

using namespace msode;

constexpr real magneticFieldMagnitude {1.0_r};

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        fprintf(stderr, "usage : %s <omega_max> <nomega> <swimmer1.cfg> <swimmer2.cfg> ... \n\n", argv[0]);
        return 1;
    }

    const double maxOmega = std::stod(argv[1]);
    const int nOmegas     = std::stoi(argv[2]);
    char **swimerNames = argv + 3;
    const int numSwimmers = argc - 3;
    std::vector<RigidBody> bodies;
    bodies.reserve(numSwimmers);

    for (int i = 0; i < numSwimmers; ++i)
        bodies.push_back(factory::readRigidBodyConfigFromFile(swimerNames[i]));

    const real dOmega = maxOmega / nOmegas;
    
    for (int i = 0; i < nOmegas; ++i)
    {
        const real omega = i * dOmega;
        for (const auto& b : bodies)
        {
            const real v = utils::computeMeanVelocityAnalytical(b, magneticFieldMagnitude, omega, 10000);
            std::cout << v << " ";
        }
        std::cout << '\n';
    }
    
    return 0;
}
