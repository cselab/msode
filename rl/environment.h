#pragma once

#include <simulation.h>

#include <memory>
#include <random>

class MSodeEnvironment
{
public:
    enum class Status {Running, MaxTimeEllapsed};
    
    MSodeEnvironment(long nstepsPerAction, real dt, std::unique_ptr<Simulation> sim, const std::vector<real3>& targetPositions);

    void reset(std::mt19937& gen);

    Status advance(const std::vector<double>& action);
    std::vector<double> getState() const;
    double getReward() const;
    
private:
    long nstepsPerAction;
    real dt, tmax {2000.0_r};
    
    std::unique_ptr<Simulation> sim;

    // magnetic field state
    real omega {0._r};
    real3 axis {1._r, 0._r, 0._r};

    std::vector<real3> targetPositions;
};
