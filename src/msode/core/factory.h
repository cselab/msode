// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "config.h"
#include "simulation.h"

#include <string>

namespace msode {
namespace factory {

/// read rigid object from a file
RigidBody readRigidBodyConfigFromFile(const std::string& fname);

/// read a RigidBody object from json format
RigidBody readRigidBodyFromConfig(const Config& config);

} // namespace factory
} // namespace msode
