#pragma once

#include "magnetic_field_from_action.h"

#include <simulation.h>

#include <memory>
#include <random>

struct Box
{
    real3 lo, hi;
    std::array<real3, 8> getCorners() const
    {
        return {real3 {lo.x, lo.y, lo.z},
                real3 {lo.x, lo.y, hi.z},
                real3 {lo.x, hi.y, lo.z},
                real3 {lo.x, hi.y, hi.z},
                real3 {hi.x, lo.y, lo.z},
                real3 {hi.x, lo.y, hi.z},
                real3 {hi.x, hi.y, lo.z},
                real3 {hi.x, hi.y, hi.z}};
    }
};

struct TimeParams
{
    real dt, tmax;
    long nstepsPerAction;
};

struct RewardParams
{
    real timeCoeff;
    real beta, K; // termination reward
    std::vector<real> multipliers;
};

struct Params
{
    Params(TimeParams time, RewardParams reward, real maxOmega, real fieldMagnitude, real distanceThreshold, Box initBox) :
        time(time),
        reward(reward),
        maxOmega(maxOmega),
        fieldMagnitude(fieldMagnitude),
        distanceThreshold(distanceThreshold),
        initBox(initBox)
    {}

    const TimeParams time;
    const RewardParams reward;
    const real maxOmega;
    const real fieldMagnitude;
    const real distanceThreshold;
    const Box initBox;
};


class MSodeEnvironment
{
public:
    enum class Status {Running, MaxTimeEllapsed, Success};
    
    MSodeEnvironment(const Params& params,
                     const std::vector<RigidBody>& initialRBs,
                     const std::vector<real3>& targetPositions);

    void reset(std::mt19937& gen);

    Status advance(const std::vector<double>& action);
    const std::vector<double>& getState() const;
    double getReward() const;

private:
    void setDistances();
    bool bodiesWithinDistanceToTargets() const;
    Status getCurrentStatus() const;

public:
    std::unique_ptr<Simulation> sim;

private:
    const long nstepsPerAction;
    const real dt;
    const real tmax;
    const real distanceThreshold;
    const Box initBox;
    const RewardParams rewardParams;
    
    //MagnFieldFromActionChange magnFieldState;
    MagnFieldFromActionDirect magnFieldState;

    std::vector<real3> targetPositions;
    mutable std::vector<real> previousDistance;
    mutable std::vector<real> cachedState;
};
