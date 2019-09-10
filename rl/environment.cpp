#include "environment.h"

MSodeEnvironment::MSodeEnvironment(long nstepsPerAction, real dt,
                                   std::unique_ptr<Simulation> sim,
                                   const std::vector<real3>& targetPositions) :
    nstepsPerAction(nstepsPerAction),
    dt(dt),
    sim(std::move(sim)),
    targetPositions(targetPositions)
{}


inline real3 randomPosition(real3 lo, real3 hi, std::mt19937& gen)
{
    std::uniform_real_distribution<real> uniformx(lo.x, hi.x);
    std::uniform_real_distribution<real> uniformy(lo.y, hi.y);
    std::uniform_real_distribution<real> uniformz(lo.z, hi.z);
    
    return {uniformx(gen), uniformy(gen), uniformz(gen)};
}

// http://planning.cs.uiuc.edu/node198.html
inline Quaternion randomOrientation(std::mt19937& gen)
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

void MSodeEnvironment::reset(std::mt19937& gen)
{
    auto field = sim->getField();
    auto bodies = sim->getBodies();

    field.phase = 0.0_r;
    
    for (auto& b : bodies)
    {
        constexpr real3 lo {-10.0_r, -10.0_r, -10.0_r};
        constexpr real3 hi {+10.0_r, +10.0_r, +10.0_r};
        b.r = randomPosition(lo, hi, gen);
        b.q = randomOrientation(gen);
    }

    sim->reset(bodies, field);
}


// smooth function [0,1] -> [0,1] with derivatives zero at edges
inline real transitionSmoothKernel(real x)
{
    // polynomial p(x) = a x^3 + b x^2
    return x * x * (3.0_r - 2.0_r * x);
}

inline real transitionLinearKernel(real x)
{
    return x;
}


MSodeEnvironment::Status MSodeEnvironment::advance(const std::vector<double>& action)
{
    // TODO: use action
    
    sim->run(nstepsPerAction, dt);

    if (sim->getCurrentTime() > tmax)
        return Status::MaxTimeEllapsed;
    
    return Status::Running;
}

std::vector<double> MSodeEnvironment::getState() const
{
    std::vector<double> state;
    const auto& bodies = sim->getBodies();
    // TODO: get states
    return state;
}

double MSodeEnvironment::getReward() const
{
    const auto& bodies = sim->getBodies();
    // TODO
}
