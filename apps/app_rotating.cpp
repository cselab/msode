#include "simulation.h"
#include "factory.h"

int main(int argc, char **argv)
{
    using namespace msode;
    
    if (argc < 2)
    {
        fprintf(stderr, "usage : ./main <config0> <config1>...");
        return 1;
    }

    std::vector<RigidBody> rigidBodies;
    
    for (int i = 1; i < argc; ++i)
        rigidBodies.push_back(Factory::readRigidBodyConfig(argv[i]));
    
    const real magneticFieldMagnitude {1.0_r};
    auto omegaField = [](real) {return 0.5_r;};

    auto rotatingDirection = [](real t)
    {
        const real omega = 0.01_r;
        const real wt_2  = 0.5_r * t * omega;
        constexpr real3 original {1._r, 0._r, 0._r};
        constexpr real3 axis {0._r, 0._r, 1._r};
        const auto q = Quaternion::createFromRotation(wt_2, axis);
        return q.rotate(original);
    };

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};
    
    Simulation simulation {rigidBodies, magneticField};

    const real tEnd = 2000.0_r;
    const real tDump = 0.1_r;
    const real dt {0.001_r};
    const long nsteps = tEnd / dt;

    simulation.activateDump("out.dat", tDump / dt);
    simulation.run(nsteps, dt);
    simulation.run(nsteps, dt);
    
    return 0;
}
