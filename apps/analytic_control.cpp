#include "simulation.h"
#include "factory.h"

#include <Eigen/LU>

#include <iostream>

using MatrixReal = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using ArrayReal  = Eigen::Array <real, Eigen::Dynamic, 1>;

constexpr real magneticFieldMagnitude {1.0_r};

static inline real computeStepOutFrequency(const RigidBody& body)
{
    const real m = length(body.magnMoment);
    const real Cxx = body.propulsion.C[0];
    return magneticFieldMagnitude * m * Cxx;
}

static inline auto computeStepOutFrequencies(const std::vector<RigidBody>& bodies)
{
    std::vector<real> omegas;
    omegas.reserve(bodies.size());
    for (const auto& b : bodies)
        omegas.push_back(computeStepOutFrequency(b));
    return omegas;
}

static inline real meanVelocity(real3 r0, real3 r1, real T)
{
    return length(r0-r1)/T;
}

static inline real computeMeanVelocity(RigidBody body, real omega)
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


static inline real computeForwardVelocity(const RigidBody& body, real omega)
{
    const real omegaC = computeStepOutFrequency(body);

    if (omega <= omegaC)
    {
        const real Bxx = body.propulsion.B[0];
        const real Cxx = body.propulsion.C[0];
        return Bxx / Cxx * omega;
    }
    else
    {
        return computeMeanVelocity(body, omega);
    }
}

static MatrixReal createVelocityMatrix(const std::vector<RigidBody>& bodies)
{
    const size_t n = bodies.size();
    const auto omegas = computeStepOutFrequencies(bodies);
    
    MatrixReal V(n, n);

    for (size_t i = 0; i < n; ++i)
    {
        const auto& body = bodies[i];
        for (size_t j = 0; j < n; ++j)
        {
            const real omega = omegas[j];
            V(i, j) = computeForwardVelocity(body, omega);
        }
    }
    return V;
}

int main(int argc, char **argv)
{
    if (argc < 2                     ||
        std::string(argv[1]) == "-h" ||
        std::string(argv[1]) == "--help")
    {
        fprintf(stderr, "usage : %s <swimmer0.cfg> <swimmer1.cfg>... \n\n", argv[0]);
        return 1;
    }

    std::vector<RigidBody> bodies;
    for (int i = 1; i < argc; ++i)
    {
        const RigidBody body = Factory::readRigidBodyConfig(argv[i]);
        bodies.push_back(body);
    }

    MatrixReal V = createVelocityMatrix(bodies);

    std::cout << V << std::endl;
    
    return 0;
}
