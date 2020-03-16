#pragma once

#include <msode/core/simulation.h>
#include <msode/core/types.h>

#include <vector>

namespace msode {
namespace rl {

class TargetDistance
{
public:
    virtual real compute(const std::vector<RigidBody>& bodies) const = 0;
};

} // namespace rl
} // namespace msode
