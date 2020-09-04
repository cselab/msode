// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "rnd.h"

namespace msode {
namespace utils {

static inline real sq(real x) {return x*x;}

real3 generateUniformPositionBox(std::mt19937& gen, real3 lo, real3 hi)
{
    std::uniform_real_distribution<real> uniformx(lo.x, hi.x);
    std::uniform_real_distribution<real> uniformy(lo.y, hi.y);
    std::uniform_real_distribution<real> uniformz(lo.z, hi.z);

    return {uniformx(gen), uniformy(gen), uniformz(gen)};
}

real3 generateUniformPositionBall(std::mt19937& gen, real radius)
{
    return generateUniformPositionShell(gen, 0.0_r, radius);
}

static inline real3 uniformUnitSphere(std::mt19937& gen)
{
    std::normal_distribution<real> normal(0.0_r, 1.0_r);
    const real3 r {normal(gen), normal(gen), normal(gen)};
    return normalized(r);
}

real3 generateUniformPositionShell(std::mt19937& gen, real r1, real r2)
{
    std::uniform_real_distribution<real> uniform(r1*r1*r1, r2*r2*r2);
    const real3 r = uniformUnitSphere(gen);
    return std::pow(uniform(gen), 1.0_r / 3.0_r) * r;
}

// http://planning.cs.uiuc.edu/node198.html
Quaternion generateUniformQuaternion(std::mt19937& gen)
{
    std::uniform_real_distribution<real> uniform(0.0_r, 1.0_r);
    const auto u1 = uniform(gen);
    const auto u2 = uniform(gen);
    const auto u3 = uniform(gen);

    constexpr real twoPi = 2 * M_PI;

    const real w = std::sqrt(1.0_r - u1) * std::sin(twoPi * u2);
    const real x = std::sqrt(1.0_r - u1) * std::cos(twoPi * u2);
    const real y = std::sqrt(u1) * std::sin(twoPi * u3);
    const real z = std::sqrt(u1) * std::cos(twoPi * u3);

    return Quaternion::createFromComponents(w, x, y, z);
}

} // namespace utils
} // namespace msode
