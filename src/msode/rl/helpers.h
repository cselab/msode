#pragma once

#include "environment.h"

#include <msode/core/factory.h>
#include <msode/core/file_parser.h>
#include <smarties.h>

#include <memory>

namespace msode {
namespace rl {

void setActionDims  (const MSodeEnvironment *env, smarties::Communicator *const comm);
void setActionBounds(const MSodeEnvironment *env, smarties::Communicator *const comm);
void setStateBounds (const MSodeEnvironment *env, smarties::Communicator *const comm);

} // namespace rl
} // namespace msode
