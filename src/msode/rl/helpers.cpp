#include "helpers.h"

#include <msode/rl/field_from_action/factory.h>
#include <msode/rl/space/factory.h>

namespace msode {
namespace rl {

std::vector<RigidBody> createBodies(const std::string& fileNameList)
{
    std::vector<RigidBody> bodies;
    const FileParser parser(fileNameList);

    for (auto entry : parser)
        bodies.push_back(factory::readRigidBodyConfigFromFile(entry.second));

    return bodies;
}

void setActionDims(const MSodeEnvironment *env, smarties::Communicator *const comm)
{
    const int nControlVars = env->numActions();
    const int nStateVars   = env->getState().size();
    comm->setStateActionDims(nStateVars, nControlVars);
}

void setActionBounds(const MSodeEnvironment *env, smarties::Communicator *const comm)
{
    const bool bounded = true;
    std::vector<double> lo, hi;
    std::tie(lo, hi) = env->getActionBounds();
    comm->setActionScales(hi, lo, bounded);
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

void setStateBounds(const std::vector<RigidBody>& bodies, const EnvSpace *spaceInfos, smarties::Communicator *const comm)
{
    std::vector<double> lo, hi;
    real3 minr {spaceInfos->target}, maxr {spaceInfos->target};
    auto min3 = [](real3 a, real3 b) {return real3{std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};};
    auto max3 = [](real3 a, real3 b) {return real3{std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};};

    minr = min3(minr, spaceInfos->getLowestPosition());
    maxr = max3(maxr, spaceInfos->getHighestPosition());

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        // quaternions, positions
        lo.insert(lo.end(), {-1.0_r, -1.0_r, -1.0_r, -1.0_r, minr.x, minr.y, minr.z});
        hi.insert(hi.end(), {+1.0_r, +1.0_r, +1.0_r, +1.0_r, maxr.x, maxr.y, maxr.z});
    }

    comm->setStateScales(hi, lo);
}

static Params createParams(const std::vector<RigidBody>& bodies, real maxDistance, real fieldMagnitude, real distanceThreshold)
{
    const int nbodies = bodies.size();
    
    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);
    const real dt       = 1.0 / (maxOmega * 20);

    const real terminationBonus = maxDistance * maxDistance * nbodies;
    
    const real timeCoeffReward = 0.1_r  * nbodies * maxDistance * computeMinForwardVelocity(fieldMagnitude, bodies);
    const real tmax            = 10.0_r * computeTimeToTravel(maxDistance, fieldMagnitude, bodies);
    const real dtAction        = 10.0_r * computeActionTimeScale(fieldMagnitude, bodies);
    const long nstepsPerAction = dtAction / dt;

    const long dumpEvery {1000}; // TODO
    // const long dumpEvery {30};
    const TimeParams timeParams {dt, tmax, nstepsPerAction, dumpEvery};
    const RewardParams rewardParams {timeCoeffReward, terminationBonus};

    fprintf(stderr,
            "----------------------------------------------------------\n"
            "tmax %g ; steps %ld ; max omega %g\n"
            "fieldMagnitude   %g\n"
            "timeCoeffReward  %g\n"
            "dt               %g\n"
            "dt action        %g\n"
            "----------------------------------------------------------\n",
            tmax, nstepsPerAction, maxOmega,
            fieldMagnitude, timeCoeffReward,
            dt, dtAction);

    const Params params(timeParams, rewardParams, fieldMagnitude, distanceThreshold);
    return params;
}

std::unique_ptr<MSodeEnvironment>
createEnvironment(const std::vector<RigidBody>& bodies, const EnvSpace *space, real fieldMagnitude, real distanceThreshold)
{
    const Params params = createParams(bodies, space->computeMaxDistanceToTarget(), fieldMagnitude, distanceThreshold);

    // using MagnFieldActionType = FieldFromActionFromLocalFrame;
    using MagnFieldActionType = FieldFromActionDirect;
    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);
    const real minOmega = 0.5_r * computeMinOmegaNoSlip(fieldMagnitude, bodies);
    auto fieldAction = std::make_unique<MagnFieldActionType>(minOmega, maxOmega);

    return std::make_unique<MSodeEnvironment>(params, space->clone(), bodies, std::move(fieldAction));
}


std::unique_ptr<MSodeEnvironment>
createEnvironmentCurriculumStateSpace(const std::vector<RigidBody>& bodies, real fieldMagnitude, real distanceThreshold, real radius, real sigmaRandomWalk)
{
    const Params params = createParams(bodies, radius, fieldMagnitude, distanceThreshold);

    auto space = std::make_unique<EnvSpaceBallCuriculumStateRW>(radius, distanceThreshold, sigmaRandomWalk);
    
    using MagnFieldActionType = FieldFromActionFromLocalFrame;
    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);
    const real minOmega = 0.5_r * computeMinOmegaNoSlip(fieldMagnitude, bodies);
    
    auto fieldAction = std::make_unique<MagnFieldActionType>(minOmega, maxOmega);
    return std::make_unique<MSodeEnvironment>(params, std::move(space), bodies, std::move(fieldAction));
}

std::unique_ptr<MSodeEnvironment>
createEnvironmentCurriculumActionSpace(const std::vector<RigidBody>& bodies, real fieldMagnitude, real distanceThreshold, real radius, real sigmaRandomWalk)
{
    const Params params = createParams(bodies, radius, fieldMagnitude, distanceThreshold);

    // used by the "pre environment" used to advance the sampled actions
    auto preEnvSpace = std::make_unique<EnvSpaceBall>(distanceThreshold);

    using MagnFieldActionType = FieldFromActionFromLocalFrame;
    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);
    const real minOmega = 0.5_r * computeMinOmegaNoSlip(fieldMagnitude, bodies);
    auto preFieldAction = std::make_unique<MagnFieldActionType>(minOmega, maxOmega);

    auto preEnvironment = std::make_unique<MSodeEnvironment>(params, std::move(preEnvSpace), bodies, std::move(preFieldAction));

    auto space = std::make_unique<EnvSpaceBallCuriculumActionRW>(std::move(preEnvironment), radius, distanceThreshold, sigmaRandomWalk);

    auto fieldAction = std::make_unique<MagnFieldActionType>(minOmega, maxOmega);
    return std::make_unique<MSodeEnvironment>(params, std::move(space), bodies, std::move(fieldAction));
}

} // namespace rl
} // namespace msode
