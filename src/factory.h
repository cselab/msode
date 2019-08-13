#pragma once

#include "simulation.h"

#include <string>

namespace Factory
{
RigidBody readRigidBodyConfig(const std::string& fname);
} // namespace Factory
