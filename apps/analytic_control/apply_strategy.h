#pragma once

#include "helpers.h"

#include <string>

namespace analytic_control
{

msode::real simulateOptimalPath(msode::real magneticFieldMagnitude,
                                std::vector<msode::RigidBody> bodies, // by copy because will be modified (IC)
                                const std::vector<msode::real3>& initialPositions,
                                const MatrixReal& U, const std::string& fname, int dumpEvery);

} // namespace analytic_control
