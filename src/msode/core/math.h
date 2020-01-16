#pragma once

#include "log.h"
#include "types.h"

#include <cmath>

namespace msode
{

inline real3& operator+=(real3& a, const real3& b)
{
    a.x += b.x;
    a.y += b.y;
    a.z += b.z;
    return a;
}

inline real3 operator+(real3 a, real3 b)
{
    a += b;
    return a;
}

inline real3& operator*=(real3& v, real a)
{
    v.x *= a;
    v.y *= a;
    v.z *= a;
    return v;
}

inline real3 operator*(real a, real3 v)
{
    return v *= a;
}

inline real3& operator-=(real3& a, const real3& b)
{
    a.x -= b.x;
    a.y -= b.y;
    a.z -= b.z;
    return a;
}

inline real3 operator-(real3 a, real3 b)
{
    a -= b;
    return a;
}

inline real dot(real3 a, real3 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

inline real3 cross(real3 a, real3 b)
{
    return {a.y * b.z - a.z * b.y,
            a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x};
}

inline real length(real3 v)
{
    return std::sqrt(dot(v, v));
}

inline real3 normalized(real3 v)
{
    const auto l = length(v);
    MSODE_Expect(l > 0, "can not normalize zero vector");
    return ( 1.0_r / l ) * v;
}

static inline real3 anyOrthogonal(real3 v)
{
    const real x = std::abs(v.x);
    const real y = std::abs(v.y);
    const real z = std::abs(v.z);

    constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
    constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
    constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};
        
    const real3 other = x < y ? (x < z ? ex : ez) : (y < z ? ey : ez);
    return cross(v, other);
}

} // namespace msode
