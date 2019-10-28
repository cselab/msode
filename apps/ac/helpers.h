#pragma once

#include "simulation.h"
#include "factory.h"

std::vector<real3> generateRandomPositions(int n, real3 boxLo, real3 boxHi, long seed = 42);
