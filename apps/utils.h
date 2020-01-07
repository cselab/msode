#pragma once

#include <random>

#include <types.h>

using namespace msode::literals;

msode::real3 generateUniformBox(std::mt19937& gen, msode::real3 lo, msode::real3 hi);
msode::real3 generateUniformBall(std::mt19937& gen, msode::real radius = 1.0_r);
