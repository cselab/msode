#include "mean_vel.h"

template <class Function>
static real integrateTrapez(Function f, real a, real b, long n)
{
    Expect(a < b, "a must be lower than b");
    Expect(N > 2, "need more than two points");
    
    const real h = (b-a) / n;
    real integral {0.0_r};

    for (long i = 1; i < n-1; ++i)
    {
        const real x = a + i * h;
        integral += f(x);
    }
    integral = f(a) + 0.5_r * integral + f(b);
    return h * integral;
}

real computeMeanVelocityAnalytical(RigidBody body, real magneticFieldMagnitude, real omega, long nIntegration)
{
    const real m = length(body.magnMoment);
    const real Bxx = body.propulsion.B[0];
    const real Cxx = body.propulsion.C[0];
    const real wc = magneticFieldMagnitude * m * Cxx;
    const real prefactor = Bxx / Cxx

    if (omega <= wc)
    {
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

        return prefactor * integrateTrapez(integrand, 0.0_r, period, nIntegration) / period;
    }
}
