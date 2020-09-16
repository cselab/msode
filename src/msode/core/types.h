// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include <iostream>

namespace msode
{

using real = double;

struct real3 {real x, y, z;};
struct int3 {int x, y, z;};

constexpr inline real3 make_real3(real a)
{
    return {a, a, a};
}

constexpr inline real3 make_real3(real x, real y, real z)
{
    return {x, y, z};
}

inline namespace literals
{
constexpr inline real operator "" _r (const long double a)
{
    return static_cast<real>(a);
}
} // namespace literals

inline std::ostream& operator<<(std::ostream& stream, const real3& v)
{
    return stream << v.x << " " << v.y << " " << v.z;
}

} // namespace msode
