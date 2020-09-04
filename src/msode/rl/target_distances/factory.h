// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

#include <msode/core/config.h>

#include <memory>

namespace msode {
namespace rl {
namespace factory {

std::unique_ptr<TargetDistance> createTargetDistance(const Config& config);

} // namespace factory
} // namespace rl
} // namespace msode
