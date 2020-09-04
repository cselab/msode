// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "mean_vel.h"
#include "integrator.h"

namespace msode {
namespace utils {

static inline real meanVelocity(real3 r0, real3 r1, real T)
{
    return length(r0-r1)/T;
}

real computeMeanVelocityODE(RigidBody body, real magneticFieldMagnitude, real omega, real tEnd)
{
    const real dt {3e-2_r / omega};
    const long nsteps = tEnd / dt;

    constexpr real3 rStart {0.0_r, 0.0_r, 0.0_r};
    body.r = rStart;

    auto omegaField        = [omega](real) {return omega;};
    auto rotatingDirection = []     (real) {return real3{1.0_r, 0.0_r, 0.0_r};};

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};
    const std::vector<RigidBody> rigidBodies {body};
    Simulation simulation {rigidBodies, magneticField};

    simulation.runRK4(nsteps, dt);

    const real3 rEnd = simulation.getBodies()[0].r;

    return meanVelocity(rStart, rEnd, tEnd);
}

real computeMeanVelocityAnalytical(RigidBody body, real magneticFieldMagnitude, real omega, long nIntegration)
{
    const real m = length(body.magnMoment);
    const real Bxx = body.propulsion.B[0];
    const real Cxx = body.propulsion.C[0];
    const real wc = magneticFieldMagnitude * m * Cxx;

    if (omega <= wc)
    {
        const real prefactor = Bxx / Cxx;
        return prefactor * omega;
    }
    else
    {
        // helper constants
        const real a1 = std::sqrt(omega*omega - wc*wc);
        const real a2 = std::atan(wc / a1);
        const real period = 2.0_r * M_PI / a1; // period for one revolution

        auto integrand = [=](real t)
        {
            // https://www.wolframalpha.com/input/?i=df%2Fdx+%3D+a*sin%28f%29+-+b%2C+f%280%29+%3D+0
            const real theta = 2.0_r * std::atan(wc / omega - a1 / omega * std::tan(0.5_r * a1 * t - a2));
            return std::sin(theta);
        };

        const real prefactor = m * magneticFieldMagnitude * Bxx / period;
        return prefactor * integrateTrapez(integrand, 0.0_r, period, nIntegration);
    }
}

} // namespace utils
} // namespace msode
