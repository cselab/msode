// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include <msode/core/simulation.h>

namespace msode {
namespace utils {

real computeMeanVelocityODE(RigidBody body, real magneticFieldMagnitude, real omega, real tend);
real computeMeanVelocityAnalytical(RigidBody body, real magneticFieldMagnitude, real omega, long nIntegration);

} // namespace utils
} // namespace msode
