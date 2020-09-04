// Copyright 2020 ETH Zurich. All Rights Reserved.
/** orient

    A small case for reorienting randomly oriented ABFs along a given axis.
    The method for reorienting consists in applying a field rotating along the target axis with alternating +/- `omega`.
    The magnitude `omega` is lower than the step out frequency to allow the rotation of the ABF.
    This is the method used used in `analytic_control`.
 */

#include <msode/simulation.h>
#include <msode/factory.h>

#include <random>
#include <string>

using namespace msode;

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

static inline Quaternion makePerpendicularOrientation(real3 dir)
{
     constexpr real3 from {1.0_r, 0.0_r, 0.0_r};
     const real3 to = normalized(anyOrthogonal(dir));
     return Quaternion::createFromVectors(from, to);
}

static inline auto readBodyAndPeropIC(const std::string& fname, int numBodies, real3 dir)
{
    const auto refBody = Factory::readRigidBodyConfig(fname);

    std::vector<RigidBody> bodies;
    bodies.reserve(numBodies);

    for (int i = 0; i < numBodies; ++i)
    {
        RigidBody b = refBody;
        b.q = makePerpendicularOrientation(dir);
        b.r = {i * 1.0_r, 0.0_r, 0.0_r};
        bodies.push_back(b);
    }

    return bodies;
}


int main(int argc, char **argv)
{
    if (argc != 6)
    {
        fprintf(stderr, "usage : ./main <config0> <numBodies> <target direction (x, y, z)>");
        return(1);
    }

    const int numBodies = std::stoi(argv[2]);

    const real3 targetDir = [argv]()
    {
        real3 dir;
        dir.x = std::stod(argv[3]);
        dir.y = std::stod(argv[4]);
        dir.z = std::stod(argv[5]);
        return normalized(dir);
    }();

    //std::vector<RigidBody> bodies = readBodyAndRandomIC(argv[1], numBodies);
    std::vector<RigidBody> bodies = readBodyAndPeropIC(argv[1], numBodies, targetDir);


    const real magneticFieldMagnitude {1.0_r};

    const auto& body = bodies[0];

    const real omegaC     = body.stepOutFrequency(magneticFieldMagnitude, 0);
    const real omegaCPerp = body.stepOutFrequency(magneticFieldMagnitude, 2);

    const real omegaTurn = omegaCPerp;

    auto omegaField = [omegaC](real t) -> real
    {
        const real period = 2.0_r * 2.0_r * M_PI / omegaC;
        const int id = t / period;
        const real sign = (id % 2) ? 1 : -1;
        return sign * omegaC * 0.5;
    };

    auto rotatingDirection = [targetDir](real) -> real3
    {
        return normalized(targetDir);
    };

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};

    Simulation simulation {bodies, magneticField};

    const real nTurns = 5;
    const int nFrames = nTurns * 50;

    const real tEnd = nTurns * 2 * M_PI / omegaTurn;
    const real tDump = tEnd / nFrames;
    const real dt    = 1.0_r / (500.0_r * omegaC);
    const long nsteps = tEnd / dt;

    simulation.activateDump("out.dat", tDump / dt);
    simulation.run(nsteps, dt);

    return 0;
}
