#pragma once

#include "interface.h"

#include <msode/analytic_control/helpers.h>

namespace msode {
namespace rl {

/** Compute the travel time from the free-space/zero velocity approximation, scaled along the principal directions.
    The scale is supposed to favor the swimmers to follow the directions one after the other in a given order.
    \note Assume that always the same bodies will be used with this object.
    \note This is not rotational invariant since the directions to follow are ex, ey, ez
 */
class TargetDistanceTravelTimeOrdered : public TargetDistance
{
public:
    /** \brief Construct a TargetDistanceTravelTimeOrdered.
        \param magneticFieldMagnitude The field magnitude that is used in the simulation. It is assumed to be constant throughout the whole simulation.
        \param scales Coefficients to multiply the travel time in each direction
     */
    TargetDistanceTravelTimeOrdered(real magneticFieldMagnitude, real3 scales);
    real compute(const std::vector<RigidBody>& bodies) const override;

private:
    const real magneticFieldMagnitude_;
    const real3 scales_;

    mutable bool initialized_ {false};
    mutable msode::analytic_control::MatrixReal U_; ///< inverse of the velocity matrix
};

} // namespace rl
} // namespace msode
