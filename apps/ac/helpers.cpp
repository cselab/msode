#include "helpers.h"

#include <random>

std::vector<real3> generateRandomPositions(int n, real3 boxLo, real3 boxHi, long seed)
{
    std::vector<real3> positions;
    positions.reserve(n);

    std::mt19937 gen(seed);
    std::uniform_real_distribution<real> distrx(boxLo.x, boxHi.x);
    std::uniform_real_distribution<real> distry(boxLo.y, boxHi.y);
    std::uniform_real_distribution<real> distrz(boxLo.z, boxHi.z);

    for (int i = 0; i < n; ++i)
    {
        real3 r;
        r.x = distrx(gen);
        r.y = distry(gen);
        r.z = distrz(gen);
        positions.push_back(r);
    }

    return positions;
}

static inline real computeStepOutFrequency(real magneticFieldMagnitude, const RigidBody& body)
{
    const real m = length(body.magnMoment);
    const real Cxx = body.propulsion.C[0];
    return magneticFieldMagnitude * m * Cxx;
}

std::vector<real> computeStepOutFrequencies(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies)
{
    std::vector<real> omegas;
    omegas.reserve(bodies.size());
    for (const auto& b : bodies)
        omegas.push_back(computeStepOutFrequency(magneticFieldMagnitude, b));
    return omegas;
}

static inline real meanVelocity(real3 r0, real3 r1, real T)
{
    return length(r0-r1)/T;
}

static inline real computeMeanVelocity(RigidBody body, real magneticFieldMagnitude, real omega)
{
    constexpr real tEnd = 200.0_r;
    constexpr real dt {1e-3_r};
    constexpr long nsteps = tEnd / dt;

    constexpr real3 rStart {0.0_r, 0.0_r, 0.0_r};
    body.r = rStart;
        
    auto omegaField        = [omega](real t) {return omega;};
    auto rotatingDirection = []     (real t) {return real3{1.0_r, 0.0_r, 0.0_r};};

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};
    const std::vector<RigidBody> rigidBodies {body};
    Simulation simulation {rigidBodies, magneticField};

    simulation.run(nsteps, dt);

    const real3 rEnd = simulation.getBodies()[0].r;

    return meanVelocity(rStart, rEnd, tEnd);
}


static inline real computeForwardVelocity(const RigidBody& body, real magneticFieldMagnitude, real omega)
{
    const real omegaC = computeStepOutFrequency(magneticFieldMagnitude, body);

    if (omega <= omegaC)
    {
        const real Bxx = body.propulsion.B[0];
        const real Cxx = body.propulsion.C[0];
        return Bxx / Cxx * omega;
    }
    else
    {
        return computeMeanVelocity(body, magneticFieldMagnitude, omega);
    }
}

MatrixReal createVelocityMatrix(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies)
{
    const size_t n = bodies.size();
    const auto omegas = computeStepOutFrequencies(magneticFieldMagnitude, bodies);
    
    MatrixReal V(n, n);

    for (size_t i = 0; i < n; ++i)
    {
        const auto& body = bodies[i];
        for (size_t j = 0; j < n; ++j)
        {
            const real omega = omegas[j];
            V(i, j) = computeForwardVelocity(body, magneticFieldMagnitude, omega);
        }
    }
    return V;
}
