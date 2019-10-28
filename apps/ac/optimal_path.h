#pragma once

#include "helpers.h"

std::vector<real3> computeA(const MatrixReal& U, const std::vector<real3>& positions);
real computeTime(real3 normal);
real3 findBestPlane(const std::vector<real3>& A);
