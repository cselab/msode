#pragma once

#include "config.h"
#include "simulation.h"

#include <string>

namespace msode
{
namespace Factory
{
/// read rigid object from a file, specific custom format; TODO: remove
RigidBody readRigidBodyConfig(const std::string& fname);

/// read a RigidBody object from json format
RigidBody readRigidBodyFromConfig(const Config& config);

} // namespace Factory
} // namespace msode
