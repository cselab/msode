#pragma once

#include "helpers.h"

#include <msode/core/quaternion.h>

namespace analytic_control
{

std::vector<msode::real3> computeA(const MatrixReal& U, const std::vector<msode::real3>& positions);


msode::real computeTime(const std::vector<msode::real3>& A, msode::real3 normal);
msode::real3 findBestPlane(const std::vector<msode::real3>& A);


msode::real computeTime(const std::vector<msode::real3>& A, msode::Quaternion q);
msode::Quaternion findBestPath(const std::vector<msode::real3>& A);

} // namespace analytic_control
