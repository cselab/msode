#include "apply_strategy.h"

#include "helpers.h"
#include "optimal_path.h"

#include <algorithm>
#include <limits>

namespace msode {
namespace analytic_control {

static inline std::vector<real> computeBetas(const std::vector<real3>& initialPositions,
                                             const MatrixReal& U, real3 dir)
{
    const size_t n = initialPositions.size();
    std::vector<real> betas(n, 0.0_r);
    
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
        {
            const real xn = dot(initialPositions[j], dir);
                betas[i] += U(i,j) * xn;
        }
    return betas;
}

static inline real computeMinOmega(int dir, const std::vector<RigidBody>& bodies, real magneticFieldMagnitude)
{
    real minVal = std::numeric_limits<real>::max();
    for (const auto& b : bodies)
        minVal = std::min(minVal, b.stepOutFrequency(magneticFieldMagnitude, dir));
    return minVal;
}

real simulateOptimalPath(real magneticFieldMagnitude,
                         std::vector<RigidBody> bodies, // by copy because will be modified (IC)
                         const std::vector<real3>& initialPositions,
                         std::unique_ptr<BaseVelocityField> velocityField,
                         const MatrixReal& U, const std::string& fname, int dumpEvery)
{
    const auto A = computeA(U, initialPositions);
    const Quaternion q = findBestPathCMAES(A);

    const std::vector<real> omegas = computeStepOutFrequencies(magneticFieldMagnitude, bodies);

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        bodies[i].r = initialPositions[i];
        bodies[i].q = q;
    }
    
    const real3 e1 {1.0_r, 0.0_r, 0.0_r};
    const real3 e2 {0.0_r, 1.0_r, 0.0_r};
    const real3 e3 {0.0_r, 0.0_r, 1.0_r};

    const real3 dir1 = q.rotate(e1);
    const real3 dir2 = q.rotate(e2);
    const real3 dir3 = q.rotate(e3);

    const auto betas1 = computeBetas(initialPositions, U, dir1);
    const auto betas2 = computeBetas(initialPositions, U, dir2);
    const auto betas3 = computeBetas(initialPositions, U, dir3);

    const real t1 = computeTravelTime(A, dir1);
    const real t2 = computeTravelTime(A, dir2);
    const real t3 = computeTravelTime(A, dir3);

    const real omegaPerpMin = computeMinOmega(2, bodies, magneticFieldMagnitude);
    constexpr real secureFactor = 5.0_r;
    const real tReorient = secureFactor * 2.0_r * M_PI / omegaPerpMin;
    const real omegaCMin = computeMinOmega(0, bodies, magneticFieldMagnitude);

    const real scan1 = tReorient + t1;
    const real scan2 = scan1 + tReorient + t2;

    auto getOmega = [&](real t, const std::vector<real>& betas)
    {
        if (t < tReorient)
        {
            const real period = 2.0_r * 2.0_r * M_PI / omegaCMin;
            const int id = t / period;
            const real sign = (id % 2) ? 1 : -1;
            return sign * omegaCMin * 0.5_r;
        }

        t -= tReorient;
        real tcum = 0.0_r;
        for (size_t i = 0; i < betas.size(); ++i)
        {
            const real beta = betas[i];
            tcum += fabs(beta);
            if (tcum > t)
            {
                return (beta > 0)
                    ? +omegas[i]
                    : -omegas[i];
            }
        }
        return 0.0_r; // default, should not happen
    };

    std::function<real(real)> omega = [&](real t)
    {
        if      (t < scan1) return getOmega(t,       betas1);
        else if (t < scan2) return getOmega(t-scan1, betas2);
        else                return getOmega(t-scan2, betas3);
    };

    std::function<real3(real)> rotatingDirection = [&](real t)
    {
        if      (t < scan1) return dir1;
        else if (t < scan2) return dir2;
        else                return dir3;
    };
    
    MagneticField magnField(magneticFieldMagnitude, omega, rotatingDirection);
    Simulation sim(bodies, magnField, std::move(velocityField));

    const real tTot = scan2 + tReorient + t3;
    const real omegaMax = *std::max_element(omegas.begin(), omegas.end());
    
    const real dt = 1.0_r / (omegaMax * 20);
    const long nsteps = static_cast<long>(tTot/dt);

    if (dumpEvery > 0)
        sim.activateDump(fname, dumpEvery);

    sim.runForwardEuler(nsteps, dt);

    return tTot;
}


real computeRequiredTime(real magneticFieldMagnitude,
                         const std::vector<RigidBody>& bodies,
                         const std::vector<real3>& initialPositions,
                         const MatrixReal& U, bool includeReorient)
{
    const auto A = computeA(U, initialPositions);
    const Quaternion q = findBestPathCMAES(A);

    const std::vector<real> omegas = computeStepOutFrequencies(magneticFieldMagnitude, bodies);

    const real3 e1 {1.0_r, 0.0_r, 0.0_r};
    const real3 e2 {0.0_r, 1.0_r, 0.0_r};
    const real3 e3 {0.0_r, 0.0_r, 1.0_r};

    const real3 dir1 = q.rotate(e1);
    const real3 dir2 = q.rotate(e2);
    const real3 dir3 = q.rotate(e3);

    const real t1 = computeTravelTime(A, dir1);
    const real t2 = computeTravelTime(A, dir2);
    const real t3 = computeTravelTime(A, dir3);

    real tReorient {0.0_r};
    if (includeReorient)
    {
        const real omegaPerpMin = computeMinOmega(2, bodies, magneticFieldMagnitude);
        constexpr real secureFactor = 5.0_r;
        tReorient = secureFactor * 2.0_r * M_PI / omegaPerpMin;
    }
    
    const real scan1 = tReorient + t1;
    const real scan2 = scan1 + tReorient + t2;
    const real tTot = scan2 + tReorient + t3;

    return tTot;
}

} // namespace analytic_control
} // namespace msode
