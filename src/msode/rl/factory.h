#pragma once

#include "environment.h"

#include <msode/core/config.h>

#include <memory>

namespace msode {
namespace rl {

std::unique_ptr<MSodeEnvironment> createEnvironment(const Config& config);

} // namespace rl
} // namespace msode
