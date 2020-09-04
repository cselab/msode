// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "helpers.h"

#include <string>

namespace msode {
namespace analytic_control {

/** Simulate the optimal path on a set of bodies with given initial conditions.
    \param magneticFieldMagnitude The magnetic field magnitude
    \param bodies The list of all RigidBody objects (positions are irrelevant)
    \param initialPositions The initial positions of each RigidBody. Must have the same dimensions as \p bodies
    \param velocityField Background velocity field.
    \param U the inverse of the velocity matrix associated to \p bodies
    \param fname Output file name of the trajectories. Only relevant if \p dumpEvery > 0
    \param dumpEvery Dump trajectory point every this amount of time steps. if <= 0, will not dump anything.
    \note The \p velocityField is just to see the effect of the field on the validity of the method. The method only solved the problem for No velocityFields
 */
real simulateOptimalPath(real magneticFieldMagnitude,
                         std::vector<RigidBody> bodies,
                         const std::vector<real3>& initialPositions,
                         std::unique_ptr<BaseVelocityField> velocityField,
                         const MatrixReal& U, const std::string& fname, int dumpEvery);

/** Compute the time taken bty the analytical method to bring the bodies to the target in free space, zero flow.
    \param magneticFieldMagnitude The magnetic field magnitude
    \param bodies The list of all RigidBody objects (positions are irrelevant)
    \param initialPositions The initial positions of each RigidBody. Must have the same dimensions as \p bodies
    \param U the inverse of the velocity matrix associated to \p bodies
    \param includeReorient \c true to include the time taken to reorient the swimmers.
 */
real computeRequiredTime(real magneticFieldMagnitude,
                         const std::vector<RigidBody>& bodies,
                         const std::vector<real3>& initialPositions,
                         const MatrixReal& U, bool includeReorient);

} // namespace analytic_control
} // namespace msode
