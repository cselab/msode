#pragma once

#include "interface.h"

#include <msode/core/config.h>

#include <memory>

namespace msode {
namespace factory {

std::unique_ptr<BaseVelocityField> createVelocityField(const Config& config);

} // namespace factory
} // namespace msode
