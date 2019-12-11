#include "analytic_control/helpers.h"
#include "analytic_control/optimal_path.h"

#include <simulation.h>

#include <iostream>
#include <random>

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

    P.B[0] = Bxx;
    P.B[1] = 0.0_r;
    P.B[2] = 0.0_r;

    P.C[0] = Cxx;
    P.C[1] = Cyy;
    P.C[2] = Czz;
    
    const real3 magnMoment {0.0_r, m, 0.0_r};
    
    const auto q = Quaternion::createFromComponents(1.0_r, 0.0_r, 0.0_r, 0.0_r);
    const real3 r {0.0_r, 0.0_r, 0.0_r};

    const RigidBody b {q, r, magnMoment, P};
    
    MSODE_Ensure(std::abs(b.stepOutFrequency(magneticFieldMagnitude, 0) - omegaC) < 1e-6_r,
                 "wrong step out frequency");
    
    return b;
}

real computeSeparability(const analytic_control::MatrixReal& V)
{
    MSODE_Expect(V.cols() == V.rows(), "Expect a square matrix");
    
    real S = 0.0_r;
    const int N = V.cols();

    for (int i = 0; i < N; ++i)
    {
        real Si = 0.0_r;
        for (int j = 0; j < N; ++j)
        {
            if (j == i) continue;
            Si += V(i,j) / V(i,i);
        }

        Si /= (N - 1);
        S += Si;
    }
    S /= N;
    return S;
}

static real computeMeanTime(const analytic_control::MatrixReal& V, const std::vector<msode::RigidBody>& bodies)
{
    const analytic_control::MatrixReal U = V.inverse();

    const real3 boxLo {-50.0_r, 0.0_r, 0.0_r};
    const real3 boxHi {+50.0_r, 0.0_r, 0.0_r};
    const real3 direction {1.0_r, 0.0_r, 0.0_r};
    
    real tSum = 0.0_r;
    const int nsamples = 500;

    for (int sample = 0; sample < nsamples; ++sample)
    {
        const long seed = 242 * sample + 13;
        auto initialPositions = analytic_control::generateRandomPositions(bodies.size(), boxLo, boxHi, seed);
        const auto A = analytic_control::computeA(U, initialPositions);
        const real t = analytic_control::computeTime(A, direction);
        tSum += t;
    }
    
    const real tMean = tSum / nsamples;
    return tMean;
}

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        fprintf(stderr, "usage : %s <number of swimmers> <omega max> <number of samples>\n\n", argv[0]);
        return 1;
    }
    
    const real Vmax = 1.0_r;
    const int nbodies = atoi(argv[1]);
    const real maxOmega = std::stod(argv[2]);
    const real minOmega = 1e-6_r;
    const int nsSamples = atoi(argv[3]);

    const long seed = 2323232323;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<real> dist(minOmega, maxOmega);

    for (int sSample = 0; sSample < nsSamples; ++sSample)
    {
        std::vector<msode::RigidBody> bodies;
    
        for (int i = 0; i < nbodies; ++i)
        {
            const real omegaC = dist(gen);
            const auto b = createRigidBody(Vmax, omegaC);
            bodies.push_back(b);
        }

        const auto V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);

        const real S = computeSeparability(V);
        const real T = computeMeanTime(V, bodies);

        printf("%g %g\n", S, T);
        fflush(stdout);
    }
    
    return 0;
}
