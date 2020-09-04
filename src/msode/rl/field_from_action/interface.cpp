// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "interface.h"

#include <msode/rl/environment.h>

namespace msode {
namespace rl {

FieldFromAction::FieldFromAction(real minOmega, real maxOmega) :
    minOmega_(minOmega),
    maxOmega_(maxOmega)
{}

void FieldFromAction::attach(const MSodeEnvironment *env)
{
    env_ = env;
}

} // namespace rl
} // namespace msode
