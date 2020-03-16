#pragma once

#include "interface.h"

#include <msode/analytic_control/helpers.h>

namespace msode {
namespace rl {

/** Compute the travel time from the free-space/zero velocity approximation.
    \note Assume that always the same bodies will be used with this object.
 */
class TargetDistanceTravelTime : public TargetDistance
{
public:
    TargetDistanceTravelTime(real magneticFieldMagnitude);
    real compute(const std::vector<RigidBody>& bodies) const override;

private:
    const real magneticFieldMagnitude_;

    mutable bool initialized_ {false};
    mutable msode::analytic_control::MatrixReal U_; ///< inverse of the velocity matrix
};

} // namespace rl
} // namespace msode
