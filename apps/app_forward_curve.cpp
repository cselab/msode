#include <msode/simulation.h>
#include <msode/factory.h>

#include "utils/mean_vel.h"

#include <iostream>

using namespace msode;

constexpr real magneticFieldMagnitude {1.0_r};

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        fprintf(stderr, "usage : %s <swimmer.cfg> <omega_max> <nomega> \n\n", argv[0]);
        return 1;
    }

    const auto body = Factory::readRigidBodyConfig(argv[1]);
    const double maxOmega = std::stod(argv[2]);
    const int nOmegas     = std::stoi(argv[3]);

    const real dOmega = maxOmega / nOmegas;
    
    for (int i = 0; i < nOmegas; ++i)
    {
        const real omega = i * dOmega;
        const real v = computeMeanVelocityODE(body, magneticFieldMagnitude, omega, 200.0_r);
        std::cout << omega << " " << v << std::endl;
    }
    
    return 0;
}
