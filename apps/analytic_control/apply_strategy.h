#pragma once

#include "helpers.h"

void simulateOptimalPath(real magneticFieldMagnitude,
                         std::vector<RigidBody> bodies, // by copy because will be modified (IC)
                         const std::vector<real3>& initialPositions,
                         const MatrixReal& U);
