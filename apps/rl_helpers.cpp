// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "rl_helpers.h"

#include <msode/rl/field_from_action/factory.h>
#include <msode/rl/pos_ic/factory.h>

using namespace msode;

void setActionDims(const rl::MSodeEnvironment *env, smarties::Communicator *const comm)
{
    const int nControlVars = env->numActions();
    const int nStateVars   = env->getState().size();
    comm->setStateActionDims(nStateVars, nControlVars);
}

void setActionBounds(const rl::MSodeEnvironment *env, smarties::Communicator *const comm)
{
    const bool bounded = true;
    std::vector<double> lo, hi;
    std::tie(lo, hi) = env->getActionBounds();
    comm->setActionScales(hi, lo, bounded);
}

static real3 min3(real3 a, real3 b)
{
    return real3{std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

static real3 max3(real3 a, real3 b)
{
    return real3{std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}


void setStateBounds(const rl::MSodeEnvironment *env, smarties::Communicator *const comm)
{
    const rl::EnvPosIC *posIc = env->getEnvPosIC();
    std::vector<double> lo, hi;

    const real3 minr = min3(posIc->target, posIc->getLowestPosition());
    const real3 maxr = max3(posIc->target, posIc->getHighestPosition());

    for (size_t i = 0; i < env->getBodies().size(); ++i)
    {
        // quaternions, positions
        lo.insert(lo.end(), {-1.0_r, -1.0_r, -1.0_r, -1.0_r, minr.x, minr.y, minr.z});
        hi.insert(hi.end(), {+1.0_r, +1.0_r, +1.0_r, +1.0_r, maxr.x, maxr.y, maxr.z});
    }

    comm->setStateScales(hi, lo);
}
