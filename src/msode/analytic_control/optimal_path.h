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
real computeTravelTime(const std::vector<real3>& A, real3 direction);

/** \brief Compute the time it takes to bring the swimmers to one point after alternating 3 perpendicular directions 
    \param A see computeA()
    \param q The rotation to generate the directions from ex, ey, ez
 */
real computeTravelTime(const std::vector<real3>& A, Quaternion q);

/** \brief Find the rotation that minimizes computeTravelTime() with CMA-ES
 */
Quaternion findBestPathCMAES(const std::vector<real3>& A);


/** \brief Compute the derivative of the travel time with respect to 3 angles.
    The angles describe the rotation as follow:
    The rotation is the one of angle \p theta around the axis u that has spherical coordinates (1, phi, psi).
 */
real3 computeTravelTimeGradient(const std::vector<real3>& A, real theta, real phi, real psi);

/** \brief Find the rotation that minimizes computeTravelTime() with LBFGS.
    \note The optimization is performed on a smoothed version of the original function.
          The absolute value is transformed to x / sqrt(x^2 + eps), with eps a small parameter
 */
Quaternion findBestPathLBFGS(const std::vector<real3>& A);

Quaternion findBestPathCMAES2(const std::vector<real3>& A);

} // namespace analytic_control
} // namespace msode
