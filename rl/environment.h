#pragma once

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
    Params(TimeParams time, RewardParams reward, real fieldMagnitude, real distanceThreshold, Box initBox) :
        time(time),
        reward(reward),
        fieldMagnitude(fieldMagnitude),
        distanceThreshold(distanceThreshold),
        initBox(initBox)
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
    
    MSodeEnvironment(const Params& params,
                     const std::vector<RigidBody>& initialRBs,
                     const std::vector<real3>& targetPositions,
                     const MagnFieldFromAction& magnFieldStateFromAction) :
        nstepsPerAction(params.time.nstepsPerAction),
        dt(params.time.dt),
        tmax(params.time.tmax),
        distanceThreshold(params.distanceThreshold),
        initBox(params.initBox),
        rewardParams(params.reward),
        magnFieldState(magnFieldStateFromAction),
        targetPositions(targetPositions)
    {
        Expect(initialRBs.size() == targetPositions.size(), "must give one target per body");

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
    
        MagneticField field{params.fieldMagnitude, omegaFunction, rotatingDirection};

        sim = std::make_unique<Simulation>(initialRBs, field);
        constexpr int dumpEvery = 1000;
        sim->activateDump("trajectories.txt", dumpEvery);
        setDistances();
    }

    int numActions() const {return magnFieldState.numActions();}
    auto getActionBounds() const {return magnFieldState.getActionBounds();}

    void reset(std::mt19937& gen)
    {
        auto field = sim->getField();
        auto bodies = sim->getBodies();

        field.phase = 0.0_r;
    
        for (auto& b : bodies)
        {
            b.r = randomPosition(initBox.lo, initBox.hi, gen);
            b.q = randomOrientation(gen);
        }

        sim->reset(bodies, field);
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
        // const auto t = sim->getCurrentTime();
        // const real3 fieldDesc = magnFieldState.getOmega(t) * magnFieldState.getAxis(t);
        cachedState.resize(0);
        // cachedState.push_back(fieldDesc.x);
        // cachedState.push_back(fieldDesc.y);
        // cachedState.push_back(fieldDesc.z);
    
        const auto& bodies = sim->getBodies();

        for (size_t i = 0; i < bodies.size(); ++i)
        {
            const real3 dr = bodies[i].r - targetPositions[i];
            cachedState.push_back(dr.x);
            cachedState.push_back(dr.y);
            cachedState.push_back(dr.z);

            const auto q = bodies[i].q;
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
    
        for (size_t i = 0; i < bodies.size(); ++i)
        {
            const real alpha = rewardParams.multipliers[i];
            const real3 dr = bodies[i].r - targetPositions[i];
            const real distance = length(dr);
        
            r += alpha * (previousDistance[i] - distance);
            previousDistance[i] = distance;

            if (status == Status::Success)
                r += alpha * rewardParams.K * std::exp(-distance*distance * rewardParams.beta);
        }
        r -= rewardParams.timeCoeff * dt * nstepsPerAction;

        return r;
    }

    const std::vector<RigidBody>& getBodies() const {return sim->getBodies();}
    const std::vector<real3>& getTargetPositions() const {return targetPositions;}


private:
    static inline real3 randomPosition(real3 lo, real3 hi, std::mt19937& gen)
    {
        std::uniform_real_distribution<real> uniformx(lo.x, hi.x);
        std::uniform_real_distribution<real> uniformy(lo.y, hi.y);
        std::uniform_real_distribution<real> uniformz(lo.z, hi.z);
    
        return {uniformx(gen), uniformy(gen), uniformz(gen)};
    }

    // http://planning.cs.uiuc.edu/node198.html
    static inline Quaternion randomOrientation(std::mt19937& gen)
    {
        std::uniform_real_distribution<real> uniform(0.0_r, 1.0_r);
        const auto u1 = uniform(gen);
        const auto u2 = uniform(gen);
        const auto u3 = uniform(gen);

        constexpr real twoPi = 2 * M_PI;
    
        const real w = std::sqrt(1.0_r - u1) * std::sin(twoPi * u2);
        const real x = std::sqrt(1.0_r - u1) * std::cos(twoPi * u2);
        const real y = std::sqrt(u1) * std::sin(twoPi * u3);
        const real z = std::sqrt(u1) * std::cos(twoPi * u3);
    
        return Quaternion::createFromComponents(w, x, y, z);
    }


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
};
