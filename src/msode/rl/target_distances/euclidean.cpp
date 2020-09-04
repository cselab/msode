// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "euclidean.h"

namespace msode {
namespace rl {

real TargetDistanceEuclidean::compute(const std::vector<RigidBody>& bodies) const
{
    real d {0.0_r};

    for (auto b : bodies)
        d += dot(b.r, b.r);

    return std::sqrt(d);
}

} // namespace rl
} // namespace msode
