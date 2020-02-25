#pragma once

#include "environment.h"
#include "field_from_action/factory.h"

#include <msode/core/factory.h>
#include <msode/core/file_parser.h>
#include <smarties.h>

#include <memory>

namespace msode {
namespace rl {

std::vector<RigidBody> createBodies(const std::string& fileNameList);

void setActionDims  (const MSodeEnvironment *env, smarties::Communicator *const comm);
void setActionBounds(const MSodeEnvironment *env, smarties::Communicator *const comm);
void setStateBounds (const MSodeEnvironment *env, smarties::Communicator *const comm);

std::unique_ptr<MSodeEnvironment>
createEnvironment(const std::vector<RigidBody>& bodies, const EnvSpace *space, real fieldMagnitude, real distanceThreshold = 2.0_r);

std::unique_ptr<MSodeEnvironment>
createEnvironmentCurriculumStateSpace(const std::vector<RigidBody>& bodies, real fieldMagnitude, real distanceThreshold, real radius, real sigmaRandomWalk);

std::unique_ptr<MSodeEnvironment>
createEnvironmentCurriculumActionSpace(const std::vector<RigidBody>& bodies, real fieldMagnitude, real distanceThreshold, real radius, real sigmaRandomWalk);

} // namespace rl
} // namespace msode
