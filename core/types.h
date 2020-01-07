#pragma once

#include <iostream>

namespace msode
{

using real = double;

struct real3 {real x, y, z;};

constexpr inline real3 make_real3(real a)
{
    return {(real)a, (real)a, (real)a};
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
