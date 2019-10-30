#include "simulation.h"
#include "factory.h"

#include <string>

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
    Expect(argc == 5, "usage : ./main <config0> <target direction (x, y, z)>");

    std::vector<RigidBody> bodies = {Factory::readRigidBodyConfig(argv[1])};

    const real3 targetDir = [argv]()
    {
        real3 dir;
        dir.x = std::stod(argv[2]);
        dir.y = std::stod(argv[3]);
        dir.z = std::stod(argv[4]);
        return normalized(dir);
    }();

    const real3 srcDir {1.0_r, 0.0_r, 0.0_r};
    
    const real magneticFieldMagnitude {1.0_r};

    const auto& body = bodies[0];
    
    const real omegaC     = computeStepOutFrequency    (magneticFieldMagnitude, body);
    const real omegaCPerp = computePerpStepOutFrequency(magneticFieldMagnitude, body);

    const real omegaTurn = 0.1_r * omegaCPerp;
    
    auto omegaField = [omegaC](real t) -> real
    {
        const real period = 2.0_r * 2.0_r * M_PI / omegaC;
        const int id = t / period;
        const real sign = (id % 2) ? 1 : -1;
        return sign * omegaC * 0.5;
    };
    
    auto rotatingDirection = [omegaTurn, targetDir, srcDir](real t) -> real3
    {
        // const real wt  = t * omegaTurn;
        // return {std::cos(wt), std::sin(wt), 0.0_r};
        const real tau = 2.0_r * M_PI / omegaTurn;
        const real lambda = std::max(0.0_r, std::min(1.0_r, t / tau));
        return normalized(lambda * targetDir + (1.0_r-lambda) * srcDir);
    };

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};

    Simulation simulation {bodies, magneticField};

    const real nTurns = 5;
    const int nFrames = nTurns * 50;
    
    const real tEnd = nTurns * 2 * M_PI / omegaTurn;
    const real tDump = tEnd / nFrames;
    const real dt    = 1.0_r / (500.0_r * omegaC);
    const long nsteps = tEnd / dt;

    simulation.activateDump("out.txt", tDump / dt);
    simulation.run(nsteps, dt);
    
    return 0;
}
