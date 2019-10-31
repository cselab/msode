#include "simulation.h"
#include "factory.h"

#include <random>
#include <string>

static inline real3 makeRandomUnitVector(std::mt19937& gen)
{
    std::uniform_real_distribution<real> U(0.0_r, 1.0_r);
    const real theta = 2.0_r * M_PI * U(gen);
    const real phi   = std::acos(1.0_r - 2.0_r * U(gen));
    return {std::sin(phi) * std::cos(theta),
            std::sin(phi) * std::sin(theta),
            std::cos(phi)};
}

static inline Quaternion makeRandomOrientation(std::mt19937& gen)
{
     constexpr real3 from {1.0_r, 0.0_r, 0.0_r};
     const real3 to = makeRandomUnitVector(gen);
     return Quaternion::createFromVectors(from, to);     
}

static inline auto readBodyAndRandomIC(const std::string& fname, int numBodies, unsigned long seed = 424242)
{
    const auto refBody = Factory::readRigidBodyConfig(fname);

    std::vector<RigidBody> bodies;
    bodies.reserve(numBodies);

    std::mt19937 gen(seed);
    
    for (int i = 0; i < numBodies; ++i)
    {
        RigidBody b = refBody;
        b.q = makeRandomOrientation(gen);
        b.r = {i * 1.0_r, 0.0_r, 0.0_r};
        bodies.push_back(b);
    }

    return bodies;
}

int main(int argc, char **argv)
{
    Expect(argc == 6, "usage : ./main <config0> <numBodies> <target direction (x, y, z)>");

    const int numBodies = std::stoi(argv[2]);
    std::vector<RigidBody> bodies = readBodyAndRandomIC(argv[1], numBodies);

    const real3 targetDir = [argv]()
    {
        real3 dir;
        dir.x = std::stod(argv[3]);
        dir.y = std::stod(argv[4]);
        dir.z = std::stod(argv[5]);
        return normalized(dir);
    }();

    const real3 srcDir {1.0_r, 0.0_r, 0.0_r};
    
    const real magneticFieldMagnitude {1.0_r};

    const auto& body = bodies[0];
    
    const real omegaC     = body.stepOutFrequency(magneticFieldMagnitude, 0);
    const real omegaCPerp = body.stepOutFrequency(magneticFieldMagnitude, 2);

    const real omegaTurn = 0.5_r * omegaCPerp;
    
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
