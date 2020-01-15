#pragma once

#include "../utils/rnd.h"
#include "magnetic_field_from_action.h"

#include "space.h"

#include <msode/simulation.h>

#include <memory>
#include <random>

using namespace msode;

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
    
    MSodeEnvironment(const Params& params_,
                     std::unique_ptr<EnvSpace>&& space_,
                     const std::vector<RigidBody>& initialRBs,
                     const std::vector<real3>& targetPositions_,
                     std::unique_ptr<MagnFieldFromActionBase>&& magnFieldStateFromAction_);
    
    MSodeEnvironment(const MSodeEnvironment&) = delete;
    MSodeEnvironment& operator=(const MSodeEnvironment&) = delete;

    MSodeEnvironment(MSodeEnvironment&&) = delete;
    MSodeEnvironment& operator=(MSodeEnvironment&&) = delete;

    int numActions() const;
    ActionBounds getActionBounds() const;

    void reset(std::mt19937& gen, long simId = NO_DUMP, bool usePreviousIC = false);
    void setPositions(const std::vector<real3>& positions);

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
    std::unique_ptr<MagnFieldFromActionBase> magnFieldState;
    const real fieldMagnitude;
    
private:
    const long nstepsPerAction;
    const real dt;
    const real tmax;
    const real distanceThreshold;
    std::unique_ptr<EnvSpace> space;
    const RewardParams rewardParams;

    std::vector<real3> targetPositions;
    mutable std::vector<real> previousDistance;
    mutable std::vector<real> cachedState;

    const long dumpEvery;
};
