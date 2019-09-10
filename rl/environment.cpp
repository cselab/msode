#include "environment.h"

MSodeEnvironment::MSodeEnvironment(real dt) :
    dt(dt)
{}

void MSodeEnvironment::reset(std::mt19937& gen)
{
    // TODO
}

MSodeEnvironment::Status MSodeEnvironment::advance(const std::vector<double>& action)
{
    // TODO
}

std::vector<double> MSodeEnvironment::getState() const
{
    // TODO
}

double MSodeEnvironment::getReward() const
{
    // TODO
}
