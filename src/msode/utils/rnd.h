#pragma once

#include <random>

#include <msode/core/quaternion.h>

using namespace msode::literals;

msode::real3 generateUniformPositionBox(std::mt19937& gen, msode::real3 lo, msode::real3 hi);
msode::real3 generateUniformPositionBall(std::mt19937& gen, msode::real radius = 1.0_r);
msode::Quaternion generateUniformQuaternion(std::mt19937& gen);
