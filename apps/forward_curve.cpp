// Copyright 2020 ETH Zurich. All Rights Reserved.
/** forward_curve

    ABF in a field rotating with frequencies `omega` in the yz plane.
    report mean velocity along the x axis for many omegas.
 */

#include <msode/core/simulation.h>
#include <msode/core/factory.h>
#include <msode/utils/mean_vel.h>

#include <iostream>

using namespace msode;

constexpr real magneticFieldMagnitude {1.0_r};

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        fprintf(stderr, "usage : %s <swimmer.json> <omega_max> <nomega> \n\n", argv[0]);
        return 1;
    }

    const auto body = factory::readRigidBodyConfigFromFile(argv[1]);
    const double maxOmega = std::stod(argv[2]);
    const int nOmegas     = std::stoi(argv[3]);

    const real dOmega = maxOmega / nOmegas;

    for (int i = 0; i < nOmegas; ++i)
    {
        const real omega = i * dOmega;
        //const real v = computeMeanVelocityODE(body, magneticFieldMagnitude, omega, 200.0_r);
        const real v = utils::computeMeanVelocityAnalytical(body, magneticFieldMagnitude, omega, 10000);
        std::cout << omega << " " << v << std::endl;
    }

    return 0;
}
