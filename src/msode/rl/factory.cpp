#include "factory.h"

#include <msode/analytic_control/optimal_path.h>
#include <msode/core/factory.h>
#include <msode/core/log.h>
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

static inline void setPositions(std::vector<RigidBody>& bodies, const std::vector<real3>& positions)
{
    MSODE_Expect(bodies.size() == positions.size(), "must have same size");
    
    for (size_t i = 0; i < positions.size(); ++i)
        bodies[i].r = positions[i];
}

static std::tuple<real, real>
estimateMaxDistanceAndTravelTime(const std::vector<RigidBody>& bodies,
                                 const EnvPosIC *posIc,
                                 const TargetDistance *targetDist,
                                 real fieldMagnitude, int nsamples = 5000)
{
    real maxDistance {0.0_r};
    real maxTravelTime {0.0_r};

    std::mt19937 gen {42422L};
    const int n = bodies.size();

    auto bodiesCopy = bodies;

    const auto V = msode::analytic_control::createVelocityMatrix(fieldMagnitude, bodies);
    const auto U = V.inverse();

    for (int i = 0; i < nsamples; ++i)
    {
        const auto positions = posIc->generateUniformPositions(gen, n);
        setPositions(bodiesCopy, positions);
        const real dist = targetDist->compute(bodiesCopy);

        const auto A = analytic_control::computeA(U, positions);
        const auto q = analytic_control::findBestPathLBFGS(A);
        const real travelTime = analytic_control::computeTravelTime(A, q);

        maxDistance = std::max(maxDistance, dist);
        maxTravelTime = std::max(maxTravelTime, travelTime);
    }
    
    return {maxDistance, maxTravelTime};
}

static Params createParams(const std::vector<RigidBody>& bodies,
                           const EnvPosIC *posIc, const TargetDistance *targetDist,
                           real fieldMagnitude, real distanceThreshold, long dumpEvery)
{
    real maxDistance, maxTravelTime;
    std::tie(maxDistance, maxTravelTime) = estimateMaxDistanceAndTravelTime(bodies, posIc, targetDist, fieldMagnitude);

    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);
    const real dt       = 1.0 / (maxOmega * 20);

    const real terminationBonus = maxDistance;
    
    const real timeCoeffReward = maxDistance / maxTravelTime;
    const real tmax            = 5.0_r * maxTravelTime;
    const real dtAction        = 20.0_r * computeActionTimeScale(fieldMagnitude, bodies);
    const long nstepsPerAction = dtAction / dt;

    const TimeParams timeParams {dt, tmax, nstepsPerAction, dumpEvery};
    const RewardParams rewardParams {timeCoeffReward, terminationBonus};

    fprintf(stderr,
            "----------------------------------------------------------\n"
            "tmax             %g\n"
            "timeCoeffReward  %g\n"
            "terminationBonus %g\n"
            "dt               %g\n"
            "dt action        %g\n"
            "steps per action %ld\n"
            "----------------------------------------------------------\n",
            tmax, timeCoeffReward, terminationBonus,
            dt, dtAction, nstepsPerAction);

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

    auto params = createParams(bodies, posIc.get(), targetDistance.get(),
                               config.at("fieldMagnitude"), config.at("targetRadius"), config.at("dumpEvery"));
    
    return std::make_unique<MSodeEnvironment>(params, std::move(posIc), bodies, std::move(fieldAction), std::move(velField), std::move(targetDistance));
}

} // namespace factory
} // namespace rl
} // namespace msode
