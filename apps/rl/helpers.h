#pragma once

#include "environment.h"
#include "magnetic_field_from_action.h"

#include <factory.h>
#include <file_parser.h>
#include <smarties.h>

#include <memory>

struct EnvSpace
{
    EnvSpace(real L_) :
        L(L_),
        domain{{-L_, -L_, -L_},
               {+L_, +L_, +L_}}
    {}

    const real L;
    const Box domain;
    const real3 target {0.0_r, 0.0_r, 0.0_r};
};

std::vector<RigidBody> createBodies(const std::string& fileNameList);
real computeMaxDistance(Box src, real3 dst);

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

void setStateBounds(const std::vector<RigidBody>& bodies, const EnvSpace& spaceInfos, smarties::Communicator *const comm);

// using MagnFieldActionType = MagnFieldFromActionDirect;
// using MagnFieldActionType = MagnFieldFromActionFromTargets;
using MagnFieldActionType = MagnFieldFromActionFromLocalFrame;
// using MagnFieldActionType = MagnFieldFromActionFromLocalPlane;

std::unique_ptr<MSodeEnvironment<MagnFieldActionType>>
createEnvironment(const std::vector<RigidBody>& bodies, const EnvSpace& space, real fieldMagnitude);
