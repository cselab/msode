#include "simulation.h"
#include "factory.h"

// assume m is along y
static inline real computeStepOutFrequency(real magneticFieldMagnitude, const RigidBody& body)
{
    Expect(std::abs(body.magnMoment.x) < 1e-6_r &&
           std::abs(body.magnMoment.z) < 1e-6_r,
           "Assume m along y");
    const real m = length(body.magnMoment);
    const real Cxx = body.propulsion.C[0];
    return magneticFieldMagnitude * m * Cxx;
}

// assume m is along y
static inline real computePerpStepOutFrequency(real magneticFieldMagnitude, const RigidBody& body)
{
    Expect(std::abs(body.magnMoment.x) < 1e-6_r &&
           std::abs(body.magnMoment.z) < 1e-6_r,
           "Assume m along y");
    const real m = length(body.magnMoment);
    const real Czz = body.propulsion.C[2];
    return magneticFieldMagnitude * m * Czz;
}


int main(int argc, char **argv)
{
    Expect(argc != 2, "usage : ./main <config0>");

    std::vector<RigidBody> bodies = {Factory::readRigidBodyConfig(argv[1])};
    
    
    const real magneticFieldMagnitude {1.0_r};
    auto omegaField = [](real t) {return 0.5_r;};

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

    Simulation simulation {bodies, magneticField};

    const real tEnd = 2000.0_r;
    const real tDump = 0.1_r;
    const real dt {0.001_r};
    const long nsteps = tEnd / dt;

    simulation.activateDump("out.txt", tDump / dt);
    simulation.run(nsteps, dt);
    simulation.run(nsteps, dt);
    
    return 0;
}
