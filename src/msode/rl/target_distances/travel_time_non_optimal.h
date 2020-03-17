#pragma once

#include "interface.h"

#include <msode/analytic_control/optimal_path.h>

namespace msode {
namespace rl {

/** Compute the travel time from the free-space/zero velocity approximation.
    \note Assume that always the same bodies will be used with this object.
    \note We do NOT compute the optimal travel time; we choose an arbitrary orientation instead.
          This allows to save computational resources.
 */
class TargetDistanceTravelTimeNonOptimal : public TargetDistance
{
public:
    TargetDistanceTravelTimeNonOptimal(real magneticFieldMagnitude);
    real compute(const std::vector<RigidBody>& bodies) const override;

private:
    const real magneticFieldMagnitude_;

    mutable bool initialized_ {false};
    mutable msode::analytic_control::MatrixReal U_; ///< inverse of the velocity matrix
    Quaternion q_{Quaternion::createIdentity()}; ///< orientation used to compute the travel time 
};

} // namespace rl
} // namespace msode
