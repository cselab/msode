#pragma once

#include "interface.h"

#include <msode/analytic_control/helpers.h>

namespace msode {
namespace rl {

/** Euclidean travel time: sqrt(sum travel times squared)
 */
class TargetDistanceEuclideanTT : public TargetDistance
{
public:
    TargetDistanceEuclideanTT(real magneticFieldMagnitude);
    real compute(const std::vector<RigidBody>& bodies) const override;

private:
    const real magneticFieldMagnitude_;

    mutable bool initialized_ {false};
    mutable msode::analytic_control::MatrixReal U_; ///< inverse of the velocity matrix
};

} // namespace rl
} // namespace msode
