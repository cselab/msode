#pragma once

#include <random>

#include <msode/core/quaternion.h>

namespace msode {
namespace utils {

real3 generateUniformPositionBox(std::mt19937& gen, real3 lo, real3 hi);
real3 generateUniformPositionBall(std::mt19937& gen, real radius = 1.0_r);
Quaternion generateUniformQuaternion(std::mt19937& gen);

} // namespace utils
} // namespace msode
