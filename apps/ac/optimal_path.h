#pragma once

#include "helpers.h"

#include <quaternion.h>

std::vector<real3> computeA(const MatrixReal& U, const std::vector<real3>& positions);


real computeTime(real3 normal);
real3 findBestPlane(const std::vector<real3>& A);


real computeTime(Quaternion q);
Quaternion findBestPath(const std::vector<real3>& A);
