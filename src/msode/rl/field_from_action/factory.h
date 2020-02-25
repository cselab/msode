#pragma once

#include "interface.h"
#include "weighted_targets.h"
#include "action_change.h"
#include "direct.h"
#include "local_frame.h"

#include <msode/core/config.h>

#include <memory>

namespace msode {
namespace rl {

std::unique_ptr<FieldFromAction> createFieldFromAction(const Config& config);

} // namespace rl
} // namespace msode
