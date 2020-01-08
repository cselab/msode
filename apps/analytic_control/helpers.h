#pragma once

#include <msode/simulation.h>
#include <msode/factory.h>

#include <Eigen/LU>

namespace analytic_control
{

using MatrixReal = Eigen::Matrix<msode::real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using ArrayReal  = Eigen::Array <msode::real, Eigen::Dynamic, 1>;

std::vector<msode::real> computeStepOutFrequencies(msode::real magneticFieldMagnitude, const std::vector<msode::RigidBody>& bodies);
std::vector<msode::real3> generateRandomPositions(int n, msode::real3 boxLo, msode::real3 boxHi, long seed = 42);
MatrixReal createVelocityMatrix(msode::real magneticFieldMagnitude, const std::vector<msode::RigidBody>& bodies);

std::vector<msode::real> computeEigenValues(const MatrixReal& A);

} // namespace analytic_control
