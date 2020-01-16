#pragma once

#include "helpers.h"

#include <string>

namespace analytic_control
{

msode::real simulateOptimalPath(msode::real magneticFieldMagnitude,
                                std::vector<msode::RigidBody> bodies, // by copy because will be modified (IC)
                                const std::vector<msode::real3>& initialPositions,
                                const MatrixReal& U, const std::string& fname, int dumpEvery);

msode::real computeRequiredTime(msode::real magneticFieldMagnitude,
                                const std::vector<msode::RigidBody>& bodies,
                                const std::vector<msode::real3>& initialPositions,
                                const MatrixReal& U);

} // namespace analytic_control
