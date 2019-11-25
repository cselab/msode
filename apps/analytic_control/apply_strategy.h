#pragma once

#include "helpers.h"

#include <string>

real simulateOptimalPath(real magneticFieldMagnitude,
                         std::vector<RigidBody> bodies, // by copy because will be modified (IC)
                         const std::vector<real3>& initialPositions,
                         const MatrixReal& U, const std::string& fname, int dump_every);
