#pragma once

#include "../utils.h"

#include <simulation.h>

#include <iomanip>
#include <memory>
#include <random>
#include <sstream>

using namespace msode;

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
    long dumpEvery;
};

struct RewardParams
{
    real timeCoeff;
    real terminationBonus;
};

struct Params
{
    Params(TimeParams time_, RewardParams reward_, real fieldMagnitude_, real distanceThreshold_, Box initBox_) :
        time(time_),
        reward(reward_),
        fieldMagnitude(fieldMagnitude_),
        distanceThreshold(distanceThreshold_),
        initBox(initBox_)
    {}

    const TimeParams time;
    const RewardParams reward;
    const real fieldMagnitude;
    const real distanceThreshold;
    const Box initBox;
};

template<class MagnFieldFromAction>
class MSodeEnvironment
{
public:
    enum class Status {Running, MaxTimeEllapsed, Success};
    
    MSodeEnvironment(const Params& params_,
                     const std::vector<RigidBody>& initialRBs,
                     const std::vector<real3>& targetPositions_,
                     const MagnFieldFromAction& magnFieldStateFromAction_) :
        nstepsPerAction(params_.time.nstepsPerAction),
        dt(params_.time.dt),
        tmax(params_.time.tmax),
        distanceThreshold(params_.distanceThreshold),
        initBox(params_.initBox),
        rewardParams(params_.reward),
        magnFieldState(magnFieldStateFromAction_),
        targetPositions(targetPositions_),
        dumpEvery(params_.time.dumpEvery)
    {
        MSODE_Expect(initialRBs.size() == targetPositions.size(), "must give one target per body");

        magnFieldState.attach(this);
        
        auto omegaFunction = [this](real t)
        {
            return magnFieldState.getOmega(t);
        };

        auto rotatingDirection = [this](real t) -> real3
        {
            const real3 axis = magnFieldState.getAxis(t);
            return normalized(axis);
        };
    
        MagneticField field{params_.fieldMagnitude, omegaFunction, rotatingDirection};

        sim = std::make_unique<Simulation>(initialRBs, field);
        setDistances();
    }

    MSodeEnvironment(const MSodeEnvironment&) = delete;
    MSodeEnvironment& operator=(const MSodeEnvironment&) = delete;

    MSodeEnvironment(MSodeEnvironment&&) = delete;
    MSodeEnvironment& operator=(MSodeEnvironment&&) = delete;

    int numActions() const {return magnFieldState.numActions();}
    auto getActionBounds() const {return magnFieldState.getActionBounds();}

    void reset(long simId, std::mt19937& gen)
    {
        auto field  = sim->getField();
        auto bodies = sim->getBodies();

        field.phase = 0.0_r;
    
        for (auto& b : bodies)
        {
            b.r = generateUniformPositionBox(gen, initBox.lo, initBox.hi);
            b.q = generateUniformQuaternion(gen);
        }

        std::ostringstream ss;
        ss << std::setw(6) << std::setfill('0') << simId;
        const std::string outputFileName = "trajectories_" + ss.str() + ".dat";

        sim->reset(bodies, field);
        sim->activateDump(outputFileName, dumpEvery);
        setDistances();
    }


    Status advance(const std::vector<double>& action)
    {
        magnFieldState.advance(sim->getCurrentTime());
        magnFieldState.setAction(action);

        for (long step = 0; step < nstepsPerAction; ++step)
        {
            sim->advance(dt);

            auto status = getCurrentStatus();
            if (status != Status::Running)
                return status;
        }
    
        return Status::Running;
    }

    const std::vector<double>& getState() const
    {
        real3 n1, n2, n3;
        std::tie(n1, n2, n3) = magnFieldState.getFrameReference();

        const RotMatrix rot = [n1,n2,n3]()
        {
            const std::array<real, 3> n1_ {n1.x, n1.y, n1.z};
            const std::array<real, 3> n2_ {n2.x, n2.y, n2.z};
            const std::array<real, 3> n3_ {n3.x, n3.y, n3.z};
            return RotMatrix{n1_, n2_, n3_};
        }();

        const auto qRot = Quaternion::createFromMatrix(rot);
        
        cachedState.resize(0);
    
        const auto& bodies = sim->getBodies();

        for (size_t i = 0; i < bodies.size(); ++i)
        {
            const real3 dr = bodies[i].r - targetPositions[i];
            cachedState.push_back(dot(dr, n1));
            cachedState.push_back(dot(dr, n2));
            cachedState.push_back(dot(dr, n3));

            const auto q = bodies[i].q * qRot;
            cachedState.push_back(q.w);
            cachedState.push_back(q.x);
            cachedState.push_back(q.y);
            cachedState.push_back(q.z);
        }

        return cachedState;
    }

    double getReward() const
    {
        real r {0.0_r};
        const auto status = getCurrentStatus();
        const auto& bodies = sim->getBodies();

        auto square = [](real a) {return a*a;};
    
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            const real3 dr = bodies[i].r - targetPositions[i];
            const real distance = length(dr);
        
            r += square(previousDistance[i]) - square(distance);
            previousDistance[i] = distance;
        }
        r -= rewardParams.timeCoeff * dt * nstepsPerAction;

        if (status == Status::Success)
            r += rewardParams.terminationBonus;

        return r;
    }

    const std::vector<RigidBody>& getBodies() const {return sim->getBodies();}
    const std::vector<real3>& getTargetPositions() const {return targetPositions;}

    real getSimulationTime() const {return sim->getCurrentTime();}

private:
    
    void setDistances()
    {
        const auto& bodies = sim->getBodies();
        previousDistance.resize(bodies.size());
    
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            const real3 dr = bodies[i].r - targetPositions[i];
            previousDistance[i] = length(dr);
        }
    }

    bool bodiesWithinDistanceToTargets() const
    {
        real maxDistance = 0.0_r;
        const auto& bodies = sim->getBodies();
    
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            const real distance = length(bodies[i].r - targetPositions[i]);
            maxDistance = std::max(maxDistance, distance);
        }
        return maxDistance < distanceThreshold;
    }

    Status getCurrentStatus() const
    {
        if (sim->getCurrentTime() > tmax)
            return Status::MaxTimeEllapsed;
        
        if (bodiesWithinDistanceToTargets())
            return Status::Success;
        
        return Status::Running;
    }

public:
    std::unique_ptr<Simulation> sim;

private:
    const long nstepsPerAction;
    const real dt;
    const real tmax;
    const real distanceThreshold;
    const Box initBox;
    const RewardParams rewardParams;
    
    MagnFieldFromAction magnFieldState;

    std::vector<real3> targetPositions;
    mutable std::vector<real> previousDistance;
    mutable std::vector<real> cachedState;

    const long dumpEvery;
};
