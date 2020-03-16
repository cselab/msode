#include "square.h"

namespace msode {
namespace rl {

real TargetDistanceSquare::compute(const std::vector<RigidBody>& bodies) const
{
    real d {0.0_r};

    for (auto b : bodies)
        d += dot(b.r, b.r);

    return d;
}

} // namespace rl
} // namespace msode
