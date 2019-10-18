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
        // TODO
        return 0.0_r;
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
    if (argc != 4)
    {
        fprintf(stderr, "usage : %s <swimmer.cfg> \n\n", argv[0]);
        return 1;
    }
    std::vector<RigidBody> bodies; // TODO    

    MatrixReal V = createVelocityMatrix(bodies);
    
    return 0;
}
