#pragma once

#include <msode/simulation.h>

using msode::RigidBody;
using msode::real;

real computeMeanVelocityAnalytical(RigidBody body, real magneticFieldMagnitude, real omega, long nIntegration);
