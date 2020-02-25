#pragma once

#include "interface.h"

#include <msode/core/config.h>

#include <memory>

namespace msode
{

std::unique_ptr<BaseVelocityField> createVelocityField(const Config& config);

} // namespace msode
