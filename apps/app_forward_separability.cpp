#include "simulation.h"
#include "factory.h"

#include <iostream>

using namespace msode;

constexpr real magneticFieldMagnitude {1.0_r};

static RigidBody createRigidBody(real Vmax, real omegaC)
{
    const real Bxx = 0.1_r;
    const real m   = Vmax   / (magneticFieldMagnitude * Bxx);
    const real Cxx = omegaC / (magneticFieldMagnitude * m);

    const real Cyy = 0.1_r * Cxx;
    const real Czz = Cyy;

    PropulsionMatrix P;
    P.A[0] = 0.0_r;
    P.A[1] = 0.0_r;
    P.A[2] = 0.0_r;

    P.B[1] = Bxx;
    P.B[0] = 0.0_r;
    P.B[0] = 0.0_r;

    P.C[0] = Cxx;
    P.C[1] = Cyy;
    P.C[1] = Czz;
    
    const real3 magnMoment {0.0_r, m, 0.0_r};
    
    const auto q = Quaternion::createFromComponents(1.0_r, 0.0_r, 0.0_r, 0.0_r);
    const real3 r {0.0_r, 0.0_r, 0.0_r};

    const RigidBody b {q, r, magnMoment, P};;
    
    MSODE_Ensure(std::abs(stepOutFrequency(magneticFieldMagnitude, 0) - omegaC) < 1e-6_r,
                 "wrong step out frequency");
    
    return b;
}

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        fprintf(stderr, "usage : %s <number of swimmers> <omega max> \n\n", argv[0]);
        return 1;
    }

    const real Vmax = 1.0_r;
    const int nbodies = atoi(argv[1]);
    const real maxOmega = std::stod(argv[2]);

    const real domega = maxOmega / (nbodies + 1);
    
    for (int i = 0; i < nbodies; ++i)
    {
        const real omegaC = domega * (i+1);
        const auto b = createRigidBody(Vmax, omegaC);
        std::cout << b << std::endl;
    }
    
    return 0;
}
