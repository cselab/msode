#pragma once

#include "helpers.h"

#include <msode/core/quaternion.h>

namespace msode {
namespace analytic_control {

/// \return A = U * positions
std::vector<real3> computeA(const MatrixReal& U, const std::vector<real3>& positions);

/** \brief Compute the time it takes to bring the swimmers on the plane perpendicular to the given direction 
    \param A see computeA()
    \param direction the normal to the plane; must be of unit length
 */
real computeTime(const std::vector<real3>& A, real3 direction);

/** \brief Compute the time it takes to bring the swimmers to one point after alternating 3 perpendicular directions 
    \param A see computeA()
    \param q The rotation to generate the directions from ex, ey, ez
 */
real computeTime(const std::vector<real3>& A, Quaternion q);

/** \brief Find the rotation that minimizes computeTime() with CMA-ES
 */
Quaternion findBestPath(const std::vector<real3>& A);

} // namespace analytic_control
} // namespace msode
