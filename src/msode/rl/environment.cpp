#include "environment.h"

#include <iomanip>
#include <sstream>

namespace msode {
namespace rl {

Params::Params(TimeParams time_, RewardParams reward_, real fieldMagnitude_, real distanceThreshold_) :
    time(time_),
    reward(reward_),
    fieldMagnitude(fieldMagnitude_),
    distanceThreshold(distanceThreshold_)
{}


MSodeEnvironment::MSodeEnvironment(const Params& params,
                                   std::unique_ptr<EnvPosIC>&& posIc,
                                   const std::vector<RigidBody>& initialRBs,
                                   std::unique_ptr<FieldFromAction>&& magnFieldStateFromAction,
                                   std::unique_ptr<BaseVelocityField>&& velocityField) :
    magnFieldState(std::move(magnFieldStateFromAction)),
    fieldMagnitude(params.fieldMagnitude),
    nstepsPerAction_(params.time.nstepsPerAction),
    dt_(params.time.dt),
    tmax_(params.time.tmax),
    distanceThreshold_(params.distanceThreshold),
    posIc_(std::move(posIc)),
    rewardParams_(params.reward),
    targetPositions_(initialRBs.size(), posIc_->target),
    dumpEvery_(params.time.dumpEvery)
{
    MSODE_Expect(initialRBs.size() == targetPositions_.size(), "must give one target per body");

    magnFieldState->attach(this);

    auto omegaFunction = [this](real t)
    {
        return magnFieldState->getOmega(t);
    };

    auto rotatingDirection = [this](real t) -> real3
    {
        const real3 axis = magnFieldState->getAxis(t);
        return normalized(axis);
    };
    
    MagneticField field{params.fieldMagnitude, omegaFunction, rotatingDirection};

    sim = std::make_unique<Simulation>(initialRBs, field, std::move(velocityField));
    setDistances();
}

int MSodeEnvironment::numActions() const
{
    return magnFieldState->numActions();
}

ActionBounds MSodeEnvironment::getActionBounds() const
{
    return magnFieldState->getActionBounds();
}

void MSodeEnvironment::reset(std::mt19937& gen, long simId, bool succesfulPreviousTry)
{
    MagneticField          field  = sim->getField();
    std::vector<RigidBody> bodies = sim->getBodies();

    field.phase = 0.0_r;

    const auto positions = posIc_->generateNewPositionsEveryMaxTries(gen, bodies.size(), succesfulPreviousTry);

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        bodies[i].r = positions[i];
        bodies[i].q = utils::generateUniformQuaternion(gen);
    }

    sim->reset(bodies, field);

    if (simId != NO_DUMP)
    {
        std::ostringstream ss;
        ss << std::setw(6) << std::setfill('0') << simId;
        const std::string outputFileName = "trajectories_" + ss.str() + ".dat";
        sim->activateDump(outputFileName, dumpEvery_);
    }
    setDistances();
}

void MSodeEnvironment::setPositions(const std::vector<real3>& positions)
{
    auto& bodies = sim->getBodies();
        
    for (size_t i = 0; i < bodies.size(); ++i)
        bodies[i].r = positions[i];
}

std::vector<real3> MSodeEnvironment::getPositions() const
{
    std::vector<real3> positions;
    auto bodies = sim->getBodies();
        
    for (auto b : bodies)
        positions.push_back(b.r);
    return positions;
}

MSodeEnvironment::Status MSodeEnvironment::advance(const std::vector<double>& action)
{
    magnFieldState->advance(sim->getCurrentTime());
    magnFieldState->setAction(action);

    for (long step = 0; step < nstepsPerAction_; ++step)
    {
        sim->advanceForwardEuler(dt_);

        auto status = getCurrentStatus();
        if (status != Status::Running)
            return status;
    }
    
    return Status::Running;
}

const std::vector<double>& MSodeEnvironment::getState() const
{
    real3 n1, n2, n3;
    std::tie(n1, n2, n3) = magnFieldState->getFrameReference();

    const RotMatrix rot = [n1,n2,n3]()
    {
        const std::array<real, 3> n1_ {n1.x, n1.y, n1.z};
        const std::array<real, 3> n2_ {n2.x, n2.y, n2.z};
        const std::array<real, 3> n3_ {n3.x, n3.y, n3.z};
        return RotMatrix{n1_, n2_, n3_};
    }();

    const auto qRot = Quaternion::createFromMatrix(rot);
        
    cachedState_.resize(0);
    
    const auto& bodies = sim->getBodies();

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        const real3 dr = bodies[i].r - targetPositions_[i];
        cachedState_.push_back(dot(dr, n1));
        cachedState_.push_back(dot(dr, n2));
        cachedState_.push_back(dot(dr, n3));

        const auto q = bodies[i].q * qRot;
        cachedState_.push_back(q.w);
        cachedState_.push_back(q.x);
        cachedState_.push_back(q.y);
        cachedState_.push_back(q.z);
    }

    return cachedState_;
}

double MSodeEnvironment::getReward() const
{
    real r {0.0_r};
    const auto status = getCurrentStatus();
    const auto& bodies = sim->getBodies();

    auto square = [](real a) {return a*a;};
    
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        const real3 dr = bodies[i].r - targetPositions_[i];
        const real distance = length(dr);
        
        r += square(previousDistance_[i]) - square(distance);
        previousDistance_[i] = distance;
    }
    r -= rewardParams_.timeCoeff * dt_ * nstepsPerAction_;

    if (status == Status::Success)
        r += rewardParams_.terminationBonus;

    return r;
}

const std::vector<RigidBody>& MSodeEnvironment::getBodies() const
{
    return sim->getBodies();
}

const std::vector<real3>& MSodeEnvironment::getTargetPositions() const
{
    return targetPositions_;
}

const EnvPosIC* MSodeEnvironment::getEnvPosIC() const
{
    return posIc_.get();
}

real MSodeEnvironment::getSimulationTime() const
{
    return sim->getCurrentTime();
}

void MSodeEnvironment::setDistances()
{
    const auto& bodies = sim->getBodies();
    previousDistance_.resize(bodies.size());
    
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        const real3 dr = bodies[i].r - targetPositions_[i];
        previousDistance_[i] = length(dr);
    }
}

bool MSodeEnvironment::bodiesWithinDistanceToTargets() const
{
    real maxDistance = 0.0_r;
    const auto& bodies = sim->getBodies();
    
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        const real distance = length(bodies[i].r - targetPositions_[i]);
        maxDistance = std::max(maxDistance, distance);
    }
    return maxDistance < distanceThreshold_;
}

MSodeEnvironment::Status MSodeEnvironment::getCurrentStatus() const
{
    if (sim->getCurrentTime() > tmax_)
        return Status::MaxTimeEllapsed;
        
    if (bodiesWithinDistanceToTargets())
        return Status::Success;
        
    return Status::Running;
}

} // namespace rl
} // namespace msode
