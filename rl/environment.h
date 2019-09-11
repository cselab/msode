#pragma once

#include <simulation.h>

#include <memory>
#include <random>

class MSodeEnvironment
{
public:
    enum class Status {Running, MaxTimeEllapsed};
    
    MSodeEnvironment(long nstepsPerAction, real dt,
                     const std::vector<RigidBody>& initialRBs,
                     const std::vector<real3>& targetPositions);

    void reset(std::mt19937& gen);

    Status advance(const std::vector<double>& action);
    std::vector<double> getState() const;
    double getReward() const;

private:
    void setDistances();
    
public:
    long nstepsPerAction;
    real dt, tmax {2000.0_r};
    
    std::unique_ptr<Simulation> sim;

    struct MagnFieldState
    {
        real lastOmega {0._r};
        real3 lastAxis {1._r, 0._r, 0._r};
        real lastActionTime {0._r};

        real dOmega {0._r};
        real3 dAxis {0._r, 0._r, 0._r};

        static constexpr real maxOmega = 10.0_r; // TODO
        
        void setAction(const std::vector<double>& action);
        real omegaActionChange(real t, real actionDt);
        real3 axisActionChange(real t, real actionDt);
        void advance(real t, real actionDt);
    };

    MagnFieldState magnFieldState;

    std::vector<real3> targetPositions;
    mutable std::vector<real> previousDistance;
};
