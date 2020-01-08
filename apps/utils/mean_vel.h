#pragma once

#include <msode/simulation.h>

using msode::RigidBody;
using msode::real;

real computeMeanVelocityODE(RigidBody body, real magneticFieldMagnitude, real omega, real tend);
real computeMeanVelocityAnalytical(RigidBody body, real magneticFieldMagnitude, real omega, long nIntegration);
