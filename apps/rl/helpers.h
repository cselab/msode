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

static auto createBodies(const std::string& fileNameList)
{
    std::vector<RigidBody> bodies;
    const FileParser parser(fileNameList);

    for (auto entry : parser)
        bodies.push_back(Factory::readRigidBodyConfig(entry.second));

    return bodies;
}

static real computeMaxDistance(Box src, real3 dst)
{
    auto distFromDst = [dst] (real3 r) {return length(r-dst);};
    real d{0.0_r};
    for (auto r : src.getCorners())
        d = std::max(d, distFromDst(r));
    return d;
}

static real computeMinForwardVelocity(real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    real v = 1e9_r;
    
    for (const auto& body : bodies)
    {
        const real vmaxBody = fieldMagnitude * length(body.magnMoment) * fabs(body.propulsion.B[0]);
        v = std::min(v, vmaxBody);
    }
    return v;
}

static real computeTimeToTravel(real maxDistance, real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    return maxDistance / computeMinForwardVelocity(fieldMagnitude, bodies);
}

static real computeActionTimeScale(real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    real wmax {0._r};
    constexpr int perpDirection = 2;

    for (const auto& body : bodies)
        wmax = std::max(wmax, body.stepOutFrequency(fieldMagnitude, perpDirection));

    return 1.0_r / wmax;
}

static real computeMaxOmegaNoSlip(real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    real wmax {0._r};

    for (const auto& body : bodies)
        wmax = std::max(wmax, body.stepOutFrequency(fieldMagnitude));

    return wmax;
}

static real computeMinOmegaNoSlip(real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    real wmin {std::numeric_limits<real>::max()};

    for (const auto& body : bodies)
        wmin = std::min(wmin, body.stepOutFrequency(fieldMagnitude));

    return wmin;
}

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

static void setStateBounds(const std::vector<RigidBody>& bodies, const EnvSpace& spaceInfos, smarties::Communicator *const comm)
{
    std::vector<double> lo, hi;
    real3 minr {spaceInfos.target}, maxr {spaceInfos.target};
    auto min3 = [](real3 a, real3 b) {return real3{std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};};
    auto max3 = [](real3 a, real3 b) {return real3{std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};};

    minr = min3(minr, spaceInfos.domain.lo);
    maxr = max3(maxr, spaceInfos.domain.hi);

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        // quaternions, positions
        lo.insert(lo.end(), {-1.0_r, -1.0_r, -1.0_r, -1.0_r, minr.x, minr.y, minr.z});
        hi.insert(hi.end(), {+1.0_r, +1.0_r, +1.0_r, +1.0_r, maxr.x, maxr.y, maxr.z});
    }
        
    comm->setStateScales(hi, lo);
}

static auto createEnvironment(const std::vector<RigidBody>& bodies, const EnvSpace& space, real fieldMagnitude)
{
    const int nbodies = bodies.size();
    
    // parameters
    
    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);
    const real minOmega = 0.5_r * computeMinOmegaNoSlip(fieldMagnitude, bodies);
    const real dt             = 1.0 / (maxOmega * 20); // s
    const real maxDistance = computeMaxDistance(space.domain, space.target);
    const real distanceThreshold = 2.0_r; // body_length

    const real terminationBonus = 10.0_r * maxDistance * nbodies;
    
    const real timeCoeffReward = 0.1_r * nbodies * space.L * computeMinForwardVelocity(fieldMagnitude, bodies);
    const real tmax            = 10.0_r  * computeTimeToTravel(maxDistance, fieldMagnitude, bodies);
    const real dtAction        = 10.0_r * computeActionTimeScale(fieldMagnitude, bodies);
    const long nstepsPerAction = dtAction / dt;

    const long dumpEvery {1000}; // TODO
    // const long dumpEvery {30};
    const TimeParams timeParams {dt, tmax, nstepsPerAction, dumpEvery};
    const RewardParams rewardParams {timeCoeffReward, terminationBonus};

    const Params params {timeParams, rewardParams, fieldMagnitude, distanceThreshold, space.domain};

    fprintf(stderr,
            "----------------------------------------------------------\n"
            "tmax %g ; steps %ld ; max omega %g\n"
            "fieldMagnitude %g\n"
            "timeCoeffReward %g\n"
            "----------------------------------------------------------\n",
            tmax, nstepsPerAction, maxOmega,
            fieldMagnitude, timeCoeffReward);
    
    const std::vector<real3> targetPositions(nbodies, space.target);

    // using MagnFieldActionType = MagnFieldFromActionDirect;
    // MagnFieldActionType magnFieldAction(minOmega, maxOmega);

    // using MagnFieldActionType = MagnFieldFromActionFromTargets;
    // MagnFieldActionType magnFieldAction(maxOmega);

    using MagnFieldActionType = MagnFieldFromActionFromLocalFrame;
    MagnFieldActionType magnFieldAction(minOmega, maxOmega);

    // using MagnFieldActionType = MagnFieldFromActionFromLocalPlane;
    // MagnFieldActionType magnFieldAction(minOmega, maxOmega);

    return std::make_unique<MSodeEnvironment<MagnFieldActionType>>(params, bodies, targetPositions, magnFieldAction);
}
