#pragma once

#include <simulation.h>
#include <factory.h>

#include <Eigen/LU>

using MatrixReal = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using ArrayReal  = Eigen::Array <real, Eigen::Dynamic, 1>;

std::vector<real> computeStepOutFrequencies(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies);
std::vector<real3> generateRandomPositions(int n, real3 boxLo, real3 boxHi, long seed = 42);
MatrixReal createVelocityMatrix(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies);
