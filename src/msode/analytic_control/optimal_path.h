#pragma once

#include "helpers.h"

#include <msode/core/quaternion.h>

namespace msode {
namespace analytic_control {

std::vector<real3> computeA(const MatrixReal& U, const std::vector<real3>& positions);

real computeTime(const std::vector<real3>& A, real3 normal);
real3 findBestPlane(const std::vector<real3>& A);

real computeTime(const std::vector<real3>& A, Quaternion q);
Quaternion findBestPath(const std::vector<real3>& A);

} // namespace analytic_control
} // namespace msode
