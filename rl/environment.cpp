#include "environment.h"

void MSodeEnvironment::MagnFieldState::setAction(const std::vector<double>& action)
{
    Expect(action.size() == 4, "expect action of size 4");
    dOmega = action[0];
    dAxis.x = action[1];
    dAxis.y = action[2];
    dAxis.z = action[3];
}

// smooth function [0,1] -> [0,1] with derivatives zero at edges
inline real transitionSmoothKernel(real x) { return x * x * (3.0_r - 2.0_r * x); }
inline real transitionLinearKernel(real x) { return x; }

real MSodeEnvironment::MagnFieldState::omegaActionChange(real t, real actionDt)
{
    const real tau = (t - lastActionTime) / actionDt;
    return transitionLinearKernel(tau) * dOmega;
}

real3 MSodeEnvironment::MagnFieldState::axisActionChange(real t, real actionDt)
{
    const real tau = (t - lastActionTime) / actionDt;
    return transitionSmoothKernel(tau) * dAxis;
}

void MSodeEnvironment::MagnFieldState::advance(real t, real actionDt)
{
    // advance
    lastOmega += omegaActionChange(t, actionDt);
    lastAxis += axisActionChange(t, actionDt);
    lastActionTime = t;

    // constraints
    lastOmega = std::max(lastOmega, 0._r);
    lastOmega = std::min(lastOmega, maxOmega);
    lastAxis = normalized(lastAxis);
}

MSodeEnvironment::MSodeEnvironment(long nstepsPerAction, real dt,
                                   const std::vector<RigidBody>& initialRBs,
                                   const std::vector<real3>& targetPositions) :
    nstepsPerAction(nstepsPerAction),
    dt(dt),
    targetPositions(targetPositions)
{
    Expect(initialRBs.size() == targetPositions.size(), "must give one target per body");

    constexpr real fieldMagnitude = 1.0_r; // hardcoded magnetic scale
    const real dtAction = nstepsPerAction * dt;
    
    auto omegaFunction = [this, dtAction](real t)
    {
        return magnFieldState.lastOmega + magnFieldState.omegaActionChange(t, dtAction);
    };

    auto rotatingDirection = [this, dtAction](real t) -> real3
    {
        const real3 axis = magnFieldState.lastAxis + magnFieldState.axisActionChange(t, dtAction);
        return normalized(axis);
    };
    
    MagneticField field{fieldMagnitude, omegaFunction, rotatingDirection};

    sim = std::make_unique<Simulation>(initialRBs, field);
    sim->activateDump("out.txt", 100);
    setDistances();
}

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
    setDistances();
}

MSodeEnvironment::Status MSodeEnvironment::advance(const std::vector<double>& action)
{
    magnFieldState.advance(sim->getCurrentTime(), nstepsPerAction * dt);
    magnFieldState.setAction(action);
        
    sim->run(nstepsPerAction, dt);

    if (sim->getCurrentTime() > tmax)
        return Status::MaxTimeEllapsed;
    
    return Status::Running;
}

std::vector<double> MSodeEnvironment::getState() const
{
    const real3 fieldDesc = magnFieldState.lastOmega * magnFieldState.lastAxis;
    std::vector<double> state = {fieldDesc.x, fieldDesc.y, fieldDesc.z};
    
    const auto& bodies = sim->getBodies();

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        const real3 dr = bodies[i].r - targetPositions[i];
        state.push_back(dr.x);
        state.push_back(dr.y);
        state.push_back(dr.z);
    }

    return state;
}

double MSodeEnvironment::getReward() const
{
    real r {0.0_r};
    
    const auto& bodies = sim->getBodies();
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        const real3 dr = bodies[i].r - targetPositions[i];
        const real distance = length(dr);
        
        r += previousDistance[i] - distance;;
        previousDistance[i] = distance;
    }
    r -= dt * nstepsPerAction;
    return r;
}

void MSodeEnvironment::setDistances()
{
    const auto& bodies = sim->getBodies();
    previousDistance.resize(bodies.size());
    
    for (size_t i = 0; i < bodies.size(); ++i)
    {
        const real3 dr = bodies[i].r - targetPositions[i];
        previousDistance[i] = length(dr);
    }
}
