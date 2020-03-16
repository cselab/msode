#include "none.h"

namespace msode {
namespace rl {

real TargetDistanceNone::compute(const std::vector<RigidBody>& /* bodies */) const
{
    return 0.0_r;
}

} // namespace rl
} // namespace msode
