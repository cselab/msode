#pragma once

#include "simulation.h"

#include <string>

namespace msode
{
namespace Factory
{
RigidBody readRigidBodyConfig(const std::string& fname);
} // namespace Factory
} // namespace msode
