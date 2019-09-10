#pragma once

#include <simulation.h>
#include <random>

class MSodeEnvironment
{
public:
    enum class Status {Running, Stopped};
    
    MSodeEnvironment(real dt);

    void reset(std::mt19937& gen);

    Status advance(const std::vector<double>& action);
    std::vector<double> getState() const;
    double getReward() const;
    
private:
    real dt; // step size for one RL step
    
    // Simulation sim;
};
