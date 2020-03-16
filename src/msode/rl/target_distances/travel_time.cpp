#include "travel_time.h"

#include <msode/analytic_control/apply_strategy.h>

namespace msode {
namespace rl {

TargetDistanceTravelTime::TargetDistanceTravelTime(real magneticFieldMagnitude) :
    magneticFieldMagnitude_(magneticFieldMagnitude)
{}

static inline std::vector<real3> getPositions(const std::vector<RigidBody>& bodies)
{
    std::vector<real3> positions;
    for (auto b : bodies)
        positions.push_back(b.r);
    return positions;
}

real TargetDistanceTravelTime::compute(const std::vector<RigidBody>& bodies) const
{
    if (!initialized_)
    {
        auto V = msode::analytic_control::createVelocityMatrix(magneticFieldMagnitude_, bodies);
        U_ = V.inverse();
        initialized_ = true;
    }
    
    constexpr bool includeReorient = false; // useless for the distance computation
    return msode::analytic_control::computeRequiredTime(magneticFieldMagnitude_,
                                                        bodies, getPositions(bodies),
                                                        U_, includeReorient);
}

} // namespace rl
} // namespace msode
