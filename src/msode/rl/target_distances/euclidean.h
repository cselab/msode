// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

namespace msode {
namespace rl {

/** The euclidean distance in state space.
    In the context of rewards, this penalizes more bodies that are far away from the target.
*/
class TargetDistanceEuclidean : public TargetDistance
{
public:
    real compute(const std::vector<RigidBody>& bodies) const override;
};

} // namespace rl
} // namespace msode
