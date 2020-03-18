#include "factory.h"

#include <msode/core/log.h>
#include <msode/core/factory.h>
#include <msode/core/velocity_field/factory.h>
#include <msode/rl/field_from_action/factory.h>
#include <msode/rl/pos_ic/factory.h>
#include <msode/rl/target_distances/factory.h>

namespace msode {
namespace rl {
namespace factory {

static std::vector<RigidBody> readBodies(const Config& config)
{
    if (!config.is_array())
        msode_die("Expected an array of bodies in config");

    std::vector<RigidBody> bodies;

    for (const auto& c : config)
        bodies.push_back(msode::factory::readRigidBodyFromConfig(c));

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

// TODO what parameters should be in the config?
static Params createParams(const std::vector<RigidBody>& bodies, real maxDistance, real fieldMagnitude, real distanceThreshold, long dumpEvery)
{
    const int nbodies = bodies.size();
    
    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);
    const real dt       = 1.0 / (maxOmega * 20);

    const real terminationBonus = maxDistance * maxDistance * nbodies;
    
    const real timeCoeffReward = 0.1_r  * nbodies * maxDistance * computeMinForwardVelocity(fieldMagnitude, bodies);
    const real tmax            = 10.0_r * computeTimeToTravel(maxDistance, fieldMagnitude, bodies);
    const real dtAction        = 10.0_r * computeActionTimeScale(fieldMagnitude, bodies);
    const long nstepsPerAction = dtAction / dt;

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

std::unique_ptr<MSodeEnvironment> createEnvironment(const Config& config)
{
    auto bodies         = readBodies(config.at("bodies"));
    auto posIc          = createEnvPosIC(config.at("posIc"));
    auto fieldAction    = createFieldFromAction(config.at("fieldAction"));
    auto targetDistance = createTargetDistance(config.at("targetDistance"));
    auto velField = msode::factory::createVelocityField(config.at("velocityField"));

    auto params = createParams(bodies, posIc->computeMaxDistanceToTarget(),
                               config.at("fieldMagnitude"), config.at("targetRadius"), config.at("dumpEvery"));
    
    return std::make_unique<MSodeEnvironment>(params, std::move(posIc), bodies, std::move(fieldAction), std::move(velField), std::move(targetDistance));
}

} // namespace factory
} // namespace rl
} // namespace msode
