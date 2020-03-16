#pragma once

#include "interface.h"

namespace msode {
namespace rl {

class TargetDistanceSquare : public TargetDistance
{
public:
    real compute(const std::vector<RigidBody>& bodies) const override;
};

} // namespace rl
} // namespace msode
