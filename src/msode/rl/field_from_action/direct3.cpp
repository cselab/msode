// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "direct3.h"

#include <msode/core/log.h>
#include <msode/core/math.h>

namespace msode {
namespace rl {

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

FieldFromActionDirect3::FieldFromActionDirect3(real maxOmega) :
    FieldFromAction(0.0_r, maxOmega)
{}

int FieldFromActionDirect3::numActions() const
{
    return 3;
}

ActionBounds FieldFromActionDirect3::getActionBounds() const
{
    return {{-maxOmega_, -maxOmega_, -maxOmega_},
            {+maxOmega_, +maxOmega_, +maxOmega_}};
}

std::tuple<real3, real3, real3> FieldFromActionDirect3::getFrameReference() const
{
    return {ex, ey, ez};
}

void FieldFromActionDirect3::setAction(const std::vector<double>& action)
{
    MSODE_Expect(static_cast<int>(action.size()) == 3, "expect action of size 3");
    constexpr real tolerance = 1e-6_r;

    real3 a {action[1], action[2], action[3]};

    omega_ = length(a);

    axis_ = omega_ < tolerance ? ex : normalized(a);
}

} // namespace rl
} // namespace msode
