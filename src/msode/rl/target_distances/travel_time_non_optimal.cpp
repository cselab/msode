#include "travel_time_non_optimal.h"

#include <msode/analytic_control/optimal_path.h>

namespace msode {
namespace rl {

TargetDistanceTravelTimeNonOptimal::TargetDistanceTravelTimeNonOptimal(real magneticFieldMagnitude) :
    magneticFieldMagnitude_(magneticFieldMagnitude)
{}

static inline std::vector<real3> getPositions(const std::vector<RigidBody>& bodies)
{
    std::vector<real3> positions;
    for (auto b : bodies)
        positions.push_back(b.r);
    return positions;
}

real TargetDistanceTravelTimeNonOptimal::compute(const std::vector<RigidBody>& bodies) const
{
    if (!initialized_)
    {
        auto V = msode::analytic_control::createVelocityMatrix(magneticFieldMagnitude_, bodies);
        U_ = V.inverse();
        initialized_ = true;
    }

    auto A = msode::analytic_control::computeA(U_, getPositions(bodies));
    const real travelTime = msode::analytic_control::computeTravelTime(A, q_);
    return travelTime;
}

} // namespace rl
} // namespace msode
