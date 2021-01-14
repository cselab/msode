// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

namespace msode {
namespace rl {

/** The sum of euclidean distances in state space.
*/
class TargetDistanceSum : public TargetDistance
{
public:
    real compute(const std::vector<RigidBody>& bodies) const override;
};

} // namespace rl
} // namespace msode
