#pragma once

#include "environment.h"

#include <msode/core/config.h>

#include <memory>

namespace msode {
namespace rl {
namespace factory {

std::unique_ptr<MSodeEnvironment> createEnvironment(const Config& config);

} // namespace factory
} // namespace rl
} // namespace msode
