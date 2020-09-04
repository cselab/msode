// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include <msode/core/simulation.h>
#include <msode/core/factory.h>

#include <Eigen/LU>

namespace msode {
namespace analytic_control {

using MatrixReal = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using ArrayReal  = Eigen::Array <real, Eigen::Dynamic, 1>;

std::vector<real> computeStepOutFrequencies(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies);
std::vector<real3> generateRandomPositionsBox(int n, real3 boxLo, real3 boxHi, long seed = 42);
MatrixReal createVelocityMatrix(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies);

std::vector<real> computeEigenValues(const MatrixReal& A);

} // namespace analytic_control
} // namespace msode
