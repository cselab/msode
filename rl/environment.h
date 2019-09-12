#pragma once

#include <simulation.h>

#include <memory>
#include <random>

struct Box
{
    real3 lo, hi;
};

struct TimeParams
{
    real dt, tmax;
    long nstepsPerAction;
};

struct Params
{
    Params(TimeParams time, real maxOmega, real distanceThreshold, Box initBox) :
        time(time),
        maxOmega(maxOmega),
        distanceThreshold(distanceThreshold),
        initBox(initBox)
    {}

    const TimeParams time;
    const real maxOmega;
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
    std::vector<double> getState() const;
    double getReward() const;

private:
    void setDistances();
    bool bodiesWithinDistanceToTargets(real threshold) const;
    
public:
    std::unique_ptr<Simulation> sim;

private:
    const long nstepsPerAction;
    const real dt;
    const real tmax;
    const real distanceThreshold;
    const Box initBox;
    

    struct MagnFieldState
    {
        MagnFieldState(real maxOmega) : maxOmega(maxOmega) {}

        const real maxOmega;
        
        real lastOmega {0._r};
        real3 lastAxis {1._r, 0._r, 0._r};
        real lastActionTime {0._r};

        real dOmega {0._r};
        real3 dAxis {0._r, 0._r, 0._r};

        void setAction(const std::vector<double>& action);
        real omegaActionChange(real t, real actionDt);
        real3 axisActionChange(real t, real actionDt);
        void advance(real t, real actionDt);
    };

    MagnFieldState magnFieldState;

    std::vector<real3> targetPositions;
    mutable std::vector<real> previousDistance;
};
