// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include <msode/core/simulation.h>
#include <msode/core/types.h>

#include <vector>

namespace msode {
namespace rl {

/** Compute a distance from the given bodies to the target (origin).
    This is used in the reward computation.
 */
class TargetDistance
{
public:
    /** \brief compute the distance given a list of bodies.
        \param bodies The list of bodies to compute the distance from.
        \return The distance to the target.
    */
    virtual real compute(const std::vector<RigidBody>& bodies) const = 0;
};

} // namespace rl
} // namespace msode
