// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "sum.h"

namespace msode {
namespace rl {

real TargetDistanceSum::compute(const std::vector<RigidBody>& bodies) const
{
    real d {0.0_r};

    for (auto b : bodies)
        d += length(b.r);

    return d;
}

} // namespace rl
} // namespace msode
