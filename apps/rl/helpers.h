#pragma once

#include "environment.h"
#include "magnetic_field_from_action.h"

#include <msode/factory.h>
#include <msode/file_parser.h>
#include <smarties.h>

#include <memory>

std::vector<RigidBody> createBodies(const std::string& fileNameList);

template<typename Env>
static void setActionDims(const Env *env, smarties::Communicator *const comm)
{
    const int nControlVars = env->numActions();
    const int nStateVars   = env->getState().size();
    comm->setStateActionDims(nStateVars, nControlVars);
}

template<typename Env>
static void setActionBounds(const Env *env, smarties::Communicator *const comm)
{
    const bool bounded = true;
    std::vector<double> lo, hi;
    std::tie(lo, hi) = env->getActionBounds();
    comm->setActionScales(hi, lo, bounded);
}

void setStateBounds(const std::vector<RigidBody>& bodies, const EnvSpace *spaceInfos, smarties::Communicator *const comm);

std::unique_ptr<MSodeEnvironment>
createEnvironment(const std::vector<RigidBody>& bodies, const EnvSpace *space, real fieldMagnitude, real distanceThreshold = 2.0_r);
