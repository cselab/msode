#pragma once

#include <msode/core/log.h>
#include <msode/core/types.h>

using msode::real;

template <class Function>
static real integrateTrapez(Function f, real a, real b, long n)
{
    MSODE_Expect(a < b, "a must be lower than b");
    MSODE_Expect(n > 2, "need more than two points");

    using namespace msode::literals;
    
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
