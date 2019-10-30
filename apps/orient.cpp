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

    const auto& body = bodies[0];
    
    const real omegaC     = computeStepOutFrequency    (magneticFieldMagnitude, body);
    const real omegaCPerp = computePerpStepOutFrequency(magneticFieldMagnitude, body);

    const real omegaTurn = 0.1_r * omegaCPerp;
    
    auto omegaField = [omegaC](real t) {return omegaC * 0.5;};

    auto rotatingDirection = [omegaTurn](real t) -> real3
    {
        const real wt  = t * omegaTurn;
        return {std::cos(wt), std::sin(wt), 0.0_r};
    };

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};

    Simulation simulation {bodies, magneticField};

    const real nTurns = 5;
    const int nFrames = nTurns * 50;
    
    const real tEnd = nTurns * 2 * M_PI / omegaTurn;
    const real tDump = tEnd / nFrames;
    const real dt    = 1.0_r / (50.0_r * omegaC);
    const long nsteps = tEnd / dt;

    simulation.activateDump("out.txt", tDump / dt);
    simulation.run(nsteps, dt);
    
    return 0;
}
