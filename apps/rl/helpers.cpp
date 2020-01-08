#include "helpers.h"


std::vector<RigidBody> createBodies(const std::string& fileNameList)
{
    std::vector<RigidBody> bodies;
    const FileParser parser(fileNameList);

    for (auto entry : parser)
        bodies.push_back(Factory::readRigidBodyConfig(entry.second));

    return bodies;
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


std::unique_ptr<MSodeEnvironment<MagnFieldActionType>>
createEnvironment(const std::vector<RigidBody>& bodies, const EnvSpace *space, real fieldMagnitude)
{
    const int nbodies = bodies.size();
    
    // parameters
    
    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);
    const real minOmega = 0.5_r * computeMinOmegaNoSlip(fieldMagnitude, bodies);
    const real dt             = 1.0 / (maxOmega * 20); // s
    const real maxDistance = space->computeMaxDistanceToTarget();
    const real distanceThreshold = 2.0_r; // body_length

    const real terminationBonus = maxDistance * maxDistance * nbodies;
    
    const real timeCoeffReward = 0.1_r * nbodies * maxDistance * computeMinForwardVelocity(fieldMagnitude, bodies);
    const real tmax            = 10.0_r  * computeTimeToTravel(maxDistance, fieldMagnitude, bodies);
    const real dtAction        = 10.0_r * computeActionTimeScale(fieldMagnitude, bodies);
    const long nstepsPerAction = dtAction / dt;

    const long dumpEvery {1000}; // TODO
    // const long dumpEvery {30};
    const TimeParams timeParams {dt, tmax, nstepsPerAction, dumpEvery};
    const RewardParams rewardParams {timeCoeffReward, terminationBonus};

    auto params = std::make_unique<Params>(timeParams, rewardParams, fieldMagnitude, distanceThreshold, space->clone());

    fprintf(stderr,
            "----------------------------------------------------------\n"
            "tmax %g ; steps %ld ; max omega %g\n"
            "fieldMagnitude %g\n"
            "timeCoeffReward %g\n"
            "----------------------------------------------------------\n",
            tmax, nstepsPerAction, maxOmega,
            fieldMagnitude, timeCoeffReward);
    
    const std::vector<real3> targetPositions(nbodies, space->target);

    // MagnFieldActionType magnFieldAction(minOmega, maxOmega);
    // MagnFieldActionType magnFieldAction(maxOmega);
    MagnFieldActionType magnFieldAction(minOmega, maxOmega);
    // MagnFieldActionType magnFieldAction(minOmega, maxOmega);

    return std::make_unique<MSodeEnvironment<MagnFieldActionType>>(std::move(params), bodies, targetPositions, magnFieldAction);
}

