// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "field_from_action/interface.h"
#include "pos_ic/interface.h"
#include "target_distances/interface.h"

#include <msode/core/simulation.h>
#include <msode/utils/rnd.h>

#include <memory>
#include <random>

namespace msode {
namespace rl {

struct TimeParams
{
    real dt;
    real tmax;
    long nstepsPerAction;
    long dumpEvery;
};

struct RewardParams
{
    real distCoeff;
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
                     std::unique_ptr<EnvPosIC> posIc,
                     const std::vector<RigidBody>& initialRBs,
                     std::unique_ptr<FieldFromAction> magnFieldStateFromAction,
                     std::unique_ptr<BaseVelocityField> velocityField,
                     std::unique_ptr<TargetDistance> targetDistance);

    MSodeEnvironment(const MSodeEnvironment&) = delete;
    MSodeEnvironment& operator=(const MSodeEnvironment&) = delete;

    MSodeEnvironment(MSodeEnvironment&&) = delete;
    MSodeEnvironment& operator=(MSodeEnvironment&&) = delete;

    int numActions() const;
    ActionBounds getActionBounds() const;

    /**
       \param simId simulation id, used to create the trajectory file name. Not relevant if dumpEvery_ is zero
     */
    void reset(std::mt19937& gen, long simId, bool succesfulPreviousTry);
    void setPositions(const std::vector<real3>& positions);
    std::vector<real3> getPositions() const;

    Status advance(const std::vector<double>& action);

    const std::vector<double>& getState() const;
    double getReward() const;

    const std::vector<RigidBody>& getBodies() const;
    const std::vector<real3>& getTargetPositions() const;
    const EnvPosIC* getEnvPosIC() const;

    real getSimulationTime() const;

private:
    void _setDistances();
    bool _bodiesWithinDistanceToTargets() const;
    Status _getCurrentStatus() const;

public:
    std::unique_ptr<Simulation> sim;
    std::unique_ptr<FieldFromAction> magnFieldState;
    const real fieldMagnitude;

private:
    const long nstepsPerAction_;
    const real dt_;
    const real tmax_;
    const real distanceThreshold_;
    std::unique_ptr<EnvPosIC> posIc_;
    std::unique_ptr<TargetDistance> targetDistance_;
    const RewardParams rewardParams_;

    std::vector<real3> targetPositions_;
    mutable real previousDistance_;
    mutable std::vector<real> cachedState_;

    const long dumpEvery_;
};

} // namespace rl
} // namespace msode
