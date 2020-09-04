// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "travel_time_ordered.h"

#include <msode/analytic_control/optimal_path.h>

namespace msode {
namespace rl {

TargetDistanceTravelTimeOrdered::TargetDistanceTravelTimeOrdered(real magneticFieldMagnitude, real3 scales) :
    magneticFieldMagnitude_(magneticFieldMagnitude),
    scales_(scales)
{}

static inline std::vector<real3> getPositions(const std::vector<RigidBody>& bodies)
{
    std::vector<real3> positions;
    for (auto b : bodies)
        positions.push_back(b.r);
    return positions;
}

real TargetDistanceTravelTimeOrdered::compute(const std::vector<RigidBody>& bodies) const
{
    if (!initialized_)
    {
        auto V = msode::analytic_control::createVelocityMatrix(magneticFieldMagnitude_, bodies);
        U_ = V.inverse();
        initialized_ = true;
    }

    auto A = msode::analytic_control::computeA(U_, getPositions(bodies));

    real tt {0.0_r};
    for (const auto& a : A)
    {
        tt +=
            scales_.x * std::abs(a.x) +
            scales_.y * std::abs(a.y) +
            scales_.z * std::abs(a.z);
    }

    return tt;
}

} // namespace rl
} // namespace msode
