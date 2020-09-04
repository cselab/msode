// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include <msode/core/math.h>
#include <msode/core/types.h>

namespace msode {
namespace utils {

namespace details {

template<class F>
real goldenSectionSearch(const F& func, real a, real b, real tol)
{
    constexpr real phiInv = 1.0_r / 1.6180339887_r; // inverse of golden ratio

    real c = b - (b - a) * phiInv;
    real d = a + (b - a) * phiInv;

    while (std::abs(c - d) > tol)
    {
        if (func(c) < func(d))
            b = d;
        else
            a = c;

        c = b - (b - a) * phiInv;
        d = a + (b - a) * phiInv;
    }

    return 0.5_r * (b + a);
}

} // namespace details

/** Perform a line search using the golden section search method
    \tparam F The function type (must be f : R+ -> R)
    \param func the function to optimize
    \param alphaMax The maximum step
    \param tol The tolerance criterion
    \return The value that maximizes func on [0, alphaMax]
 */
template<class F>
real lineSearchGoldenSection(const F& func, real alphaMax, real tol = 1e-5_r)
{
    return details::goldenSectionSearch(func, 0.0_r, alphaMax, tol);
}

} // namespace utils
} // namespace msode
