#pragma once

#include "interface.h"
#include "ball.h"
#include "ball_curriculum_state.h"
#include "ball_curriculum_action.h"
#include "box.h"

#include <msode/core/config.h>

#include <memory>

namespace msode {
namespace rl {
namespace factory {

std::unique_ptr<EnvSpace> createEnvSpace(const Config& config);

} // namespace factory
} // namespace rl
} // namespace msode
