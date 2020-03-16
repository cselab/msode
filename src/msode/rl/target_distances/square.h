#pragma once

#include "interface.h"

namespace msode {
namespace rl {

/** The sum of squares of distances.
    In the context of rewards, this penalizes more bodies that are far away from the target.
*/
class TargetDistanceSquare : public TargetDistance
{
public:
    real compute(const std::vector<RigidBody>& bodies) const override;
};

} // namespace rl
} // namespace msode
