#include "helpers.h"

#include <msode/utils/mean_vel.h>

#include <Eigen/Eigenvalues>
#include <random>

using namespace msode;

namespace analytic_control
{

std::vector<real3> generateRandomPositionsBox(int n, real3 boxLo, real3 boxHi, long seed)
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

std::vector<real> computeStepOutFrequencies(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies)
{
    std::vector<real> omegas;
    omegas.reserve(bodies.size());
    for (const auto& b : bodies)
        omegas.push_back(b.stepOutFrequency(magneticFieldMagnitude));
    return omegas;
}

MatrixReal createVelocityMatrix(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies)
{
    constexpr long nIntegration = 10000;
    const size_t n = bodies.size();
    const auto omegas = computeStepOutFrequencies(magneticFieldMagnitude, bodies);
    
    MatrixReal V(n, n);

    for (size_t i = 0; i < n; ++i)
    {
        const auto& body = bodies[i];
        for (size_t j = 0; j < n; ++j)
        {
            const real omega = omegas[j];
            V(i, j) = computeMeanVelocityAnalytical(body, magneticFieldMagnitude, omega, nIntegration);
        }
    }
    return V;
}


std::vector<real> computeEigenValues(const MatrixReal& A)
{
    std::vector<real> ev;
    Eigen::EigenSolver<analytic_control::MatrixReal> es(A);
    const auto eigenValues = es.eigenvalues();

    for (auto l : eigenValues)
        ev.push_back(l.real());
    return ev;
}

} // namespace analytic_control