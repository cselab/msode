// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

#include <msode/core/config.h>

#include <memory>

namespace msode {
namespace rl {
namespace factory {

std::unique_ptr<EnvPosIC> createEnvPosIC(const Config& rootConfig, const ConfPointer& confPointer);

} // namespace factory
} // namespace rl
} // namespace msode
