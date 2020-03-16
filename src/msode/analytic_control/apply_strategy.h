#pragma once

#include "helpers.h"

#include <string>

namespace msode {
namespace analytic_control {

real simulateOptimalPath(real magneticFieldMagnitude,
                         std::vector<RigidBody> bodies, // by copy because will be modified (IC)
                         const std::vector<real3>& initialPositions,
                         std::unique_ptr<BaseVelocityField> velocityField,
                         const MatrixReal& U, const std::string& fname, int dumpEvery);

real computeRequiredTime(real magneticFieldMagnitude,
                         const std::vector<RigidBody>& bodies,
                         const std::vector<real3>& initialPositions,
                         const MatrixReal& U);

} // namespace analytic_control
} // namespace msode
