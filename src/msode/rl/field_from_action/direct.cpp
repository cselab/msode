// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "direct.h"

#include <msode/core/log.h>
#include <msode/core/math.h>

namespace msode {
namespace rl {

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

FieldFromActionDirect::FieldFromActionDirect(real minOmega, real maxOmega) :
    FieldFromAction(minOmega, maxOmega)
{}

int FieldFromActionDirect::numActions() const
{
    return 4;
}

ActionBounds FieldFromActionDirect::getActionBounds() const
{
    return {{minOmega_, -1.0, -1.0, -1.0},
            {maxOmega_,  1.0,  1.0,  1.0}};
}

std::tuple<real3, real3, real3> FieldFromActionDirect::getFrameReference() const
{
    return {ex, ey, ez};
}

void FieldFromActionDirect::setAction(const std::vector<double>& action)
{
    MSODE_Expect(static_cast<int>(action.size()) == 4, "expect action of size 4");
    constexpr real tolerance = 1e-6_r;

    omega_ = std::min(maxOmega_, std::max(minOmega_, static_cast<real>(action[0])));
    axis_.x = action[1];
    axis_.y = action[2];
    axis_.z = action[3];
    if (dot(axis_, axis_) < tolerance)
        axis_.x = 1.0_r;
    axis_ = normalized(axis_);
}

} // namespace rl
} // namespace msode
