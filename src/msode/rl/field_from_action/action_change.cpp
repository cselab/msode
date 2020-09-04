// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "action_change.h"

#include <msode/core/log.h>
#include <msode/core/math.h>

namespace msode {
namespace rl {

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

FieldFromActionChange::FieldFromActionChange(real minOmega, real maxOmega, real actionDt) :
    FieldFromAction(minOmega, maxOmega),
    actionDt_(actionDt)
{}

int FieldFromActionChange::numActions() const
{
    return 4;
}

ActionBounds FieldFromActionChange::getActionBounds() const
{
    return {{minOmega_, -1.0, -1.0, -1.0},
            {maxOmega_,  1.0,  1.0,  1.0}};
}

std::tuple<real3, real3, real3> FieldFromActionChange::getFrameReference() const
{
    return {ex, ey, ez};
}

void FieldFromActionChange::setAction(const std::vector<double>& action)
{
    MSODE_Expect(static_cast<int>(action.size()) == 4, "expect action of size 4");
    dOmega_ = action[0];
    dAxis_.x = action[1];
    dAxis_.y = action[2];
    dAxis_.z = action[3];
}

void FieldFromActionChange::advance(real t)
{
    // advance
    lastOmega_ += _omegaActionChange(t);
    lastAxis_  += _axisActionChange(t);
    lastActionTime_ = t;

    // constraints
    lastOmega_ = std::max(lastOmega_, minOmega_);
    lastOmega_ = std::min(lastOmega_, maxOmega_);
    lastAxis_ = normalized(lastAxis_);
}

real FieldFromActionChange::getOmega(real t) const
{
    return lastOmega_ + _omegaActionChange(t);
}

real3 FieldFromActionChange::getAxis(real t) const
{
    return lastAxis_ + _axisActionChange(t);
}

static inline real transitionSmoothKernel(real x) { return x * x * (3.0_r - 2.0_r * x); }
static inline real transitionLinearKernel(real x) { return x; }

real FieldFromActionChange::_omegaActionChange(real t) const
{
    const real tau = (t - lastActionTime_) / actionDt_;
    return transitionLinearKernel(tau) * dOmega_;
}

real3 FieldFromActionChange::_axisActionChange(real t) const
{
    const real tau = (t - lastActionTime_) / actionDt_;
    return transitionSmoothKernel(tau) * dAxis_;
}


} // namespace rl
} // namespace msode
