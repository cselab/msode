// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

namespace msode {
namespace rl {

/** Special Distance which always returns zero.
    Useful to test "non-engineered rewards"
 */
class TargetDistanceNone : public TargetDistance
{
public:
    real compute(const std::vector<RigidBody>& bodies) const override;
};

} // namespace rl
} // namespace msode
