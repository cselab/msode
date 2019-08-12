#pragma once

using real = double;

struct real3 {real x, y, z;};

constexpr inline real3 make_real3(real a)
{
    return {(real)a, (real)a, (real)a};
}

constexpr inline real operator "" _r (const long double a)
{
    return (real) a;
}

