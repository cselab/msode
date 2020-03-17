#pragma once

#include "line_search.h"

#include <tuple>

namespace msode {
namespace utils {

template<class F, class DerF, class Point>
std::tuple<Point, real> minimizeGradientDescent(const F& func, const DerF& derF, Point x, int maxIter, real tol)
{
    constexpr real maxAlpha = 1.0_r;
    
    Point grad;
    real fValCurr = func(x);
    real fValPrev = fValCurr;
    

    auto lineSearchFunc = [&](real alpha)
    {
        return func(x - alpha * grad);
    };
    
    for (int i = 0; i < maxIter; ++i)
    {
        grad = derF(x);
        const real alpha = lineSearchGoldenSection(lineSearchFunc, maxAlpha, tol);
        x -= alpha * grad;

        fValCurr = func(x);

        // printf("%d %g %g\n", i, alpha, fValCurr);

        if (std::abs(fValCurr - fValPrev) < tol)
            break;

        fValPrev = fValCurr;
    }
    return {x, fValCurr};
}

} // namespace utils
} // namespace msode

