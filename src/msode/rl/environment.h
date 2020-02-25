#pragma once

#include "field_from_action/interface.h"
#include "space/interface.h"

#include <msode/core/simulation.h>
#include <msode/utils/rnd.h>

#include <memory>
#include <random>

namespace msode {
namespace rl {

struct TimeParams
{
    real dt, tmax;
    long nstepsPerAction;
    long dumpEvery;
};

struct RewardParams
{
    real timeCoeff;
    real terminationBonus;
};

struct Params
{
    Params(TimeParams time_, RewardParams reward_, real fieldMagnitude_, real distanceThreshold_);

    const TimeParams time;
    const RewardParams reward;
    const real fieldMagnitude;
    const real distanceThreshold;
};

class MSodeEnvironment
{
public:
    enum class Status {Running, MaxTimeEllapsed, Success};
    enum {NO_DUMP = -1};
    
    MSodeEnvironment(const Params& params,
                     std::unique_ptr<EnvSpace>&& space,
                     const std::vector<RigidBody>& initialRBs,
                     const std::vector<real3>& targetPositions,
                     std::unique_ptr<FieldFromAction>&& magnFieldStateFromAction);
    
    MSodeEnvironment(const MSodeEnvironment&) = delete;
    MSodeEnvironment& operator=(const MSodeEnvironment&) = delete;

    MSodeEnvironment(MSodeEnvironment&&) = delete;
    MSodeEnvironment& operator=(MSodeEnvironment&&) = delete;

    int numActions() const;
    ActionBounds getActionBounds() const;

    void reset(std::mt19937& gen, long simId = NO_DUMP, bool usePreviousIC = false);
    void setPositions(const std::vector<real3>& positions);
    std::vector<real3> getPositions() const;

    Status advance(const std::vector<double>& action);

    const std::vector<double>& getState() const;
    double getReward() const;

    const std::vector<RigidBody>& getBodies() const;
    const std::vector<real3>& getTargetPositions() const;

    real getSimulationTime() const;

private:
    
    void setDistances();
    bool bodiesWithinDistanceToTargets() const;
    Status getCurrentStatus() const;

public:
    std::unique_ptr<Simulation> sim;
    std::unique_ptr<FieldFromAction> magnFieldState;
    const real fieldMagnitude;
    
private:
    const long nstepsPerAction_;
    const real dt_;
    const real tmax_;
    const real distanceThreshold_;
    std::unique_ptr<EnvSpace> space_;
    const RewardParams rewardParams_;

    std::vector<real3> targetPositions_;
    mutable std::vector<real> previousDistance_;
    mutable std::vector<real> cachedState_;

    const long dumpEvery_;
};

} // namespace rl
} // namespace msode
