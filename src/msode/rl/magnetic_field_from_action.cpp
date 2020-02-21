#include "magnetic_field_from_action.h"

#include "environment.h"

namespace msode {
namespace rl {

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

MagnFieldFromActionBase::MagnFieldFromActionBase(real minOmega, real maxOmega) :
    minOmega_(minOmega),
    maxOmega_(maxOmega)
{}

void MagnFieldFromActionBase::attach(const MSodeEnvironment *env)
{
    env_ = env;
}

MagnFieldFromActionChange::MagnFieldFromActionChange(real minOmega, real maxOmega, real actionDt) :
    MagnFieldFromActionBase(minOmega, maxOmega),
    actionDt_(actionDt)
{}

int MagnFieldFromActionChange::numActions() const
{
    return 4;
}

ActionBounds MagnFieldFromActionChange::getActionBounds() const
{
    return {{minOmega_, -1.0, -1.0, -1.0},
            {maxOmega_,  1.0,  1.0,  1.0}};
}

std::tuple<real3, real3, real3> MagnFieldFromActionChange::getFrameReference() const
{
    return {ex, ey, ez};
}

void MagnFieldFromActionChange::setAction(const std::vector<double>& action)
{
    MSODE_Expect(static_cast<int>(action.size()) == 4, "expect action of size 4");
    dOmega_ = action[0];
    dAxis_.x = action[1];
    dAxis_.y = action[2];
    dAxis_.z = action[3];
}

void MagnFieldFromActionChange::advance(real t) 
{
    // advance
    lastOmega_ += _omegaActionChange(t);
    lastAxis_  += _axisActionChange(t);
    lastActionTime_ = t;

    // constraints
    lastOmega_ = std::max(lastOmega_, minOmega_);
    lastOmega_ = std::min(lastOmega_, maxOmega_);
    lastAxis_ = normalized(lastAxis_);
}

real MagnFieldFromActionChange::getOmega(real t) const
{
    return lastOmega_ + _omegaActionChange(t);
}

real3 MagnFieldFromActionChange::getAxis(real t) const
{
    return lastAxis_ + _axisActionChange(t);
}

static inline real transitionSmoothKernel(real x) { return x * x * (3.0_r - 2.0_r * x); }
static inline real transitionLinearKernel(real x) { return x; }

real MagnFieldFromActionChange::_omegaActionChange(real t) const
{
    const real tau = (t - lastActionTime_) / actionDt_;
    return transitionLinearKernel(tau) * dOmega_;
}

real3 MagnFieldFromActionChange::_axisActionChange(real t) const
{
    const real tau = (t - lastActionTime_) / actionDt_;
    return transitionSmoothKernel(tau) * dAxis_;
}




MagnFieldFromActionDirect::MagnFieldFromActionDirect(real minOmega, real maxOmega) :
    MagnFieldFromActionBase(minOmega, maxOmega)
{}

int MagnFieldFromActionDirect::numActions() const
{
    return 4;
}

ActionBounds MagnFieldFromActionDirect::getActionBounds() const
{
    return {{minOmega_, -1.0, -1.0, -1.0},
            {maxOmega_,  1.0,  1.0,  1.0}};
}

std::tuple<real3, real3, real3> MagnFieldFromActionDirect::getFrameReference() const
{
    return {ex, ey, ez};
}
    
void MagnFieldFromActionDirect::setAction(const std::vector<double>& action) 
{
    MSODE_Expect(static_cast<int>(action.size()) == 4, "expect action of size 4");
    constexpr real tolerance = 1e-6_r;
    
    omega_ = std::min(maxOmega_, std::max(minOmega_, static_cast<real>(action[0])));
    axis_.x = action[1];
    axis_.y = action[2];
    axis_.z = action[3];
    if (dot(axis_, axis_) < tolerance)
        axis_.x = 1.0_r;
    axis_ = normalized(axis_);
}

MagnFieldFromActionFromTargets::MagnFieldFromActionFromTargets(real maxOmega) :
    MagnFieldFromActionBase(0.0, maxOmega)
{}

int MagnFieldFromActionFromTargets::numActions() const {return 1 + env_->getBodies().size();}

ActionBounds MagnFieldFromActionFromTargets::getActionBounds() const
{
    std::vector<double> lo{-maxOmega_}, hi{+maxOmega_};

    for (size_t i = 0; i < env_->getBodies().size(); ++i)
    {
        lo.push_back(0.0_r);
        hi.push_back(1.0_r);
    }
        
    return {std::move(lo), std::move(hi)};
}

std::tuple<real3, real3, real3> MagnFieldFromActionFromTargets::getFrameReference() const
{
    return {ex, ey, ez};
}
    
void MagnFieldFromActionFromTargets::setAction(const std::vector<double>& action)
{
    MSODE_Expect(static_cast<int>(action.size()) == numActions(),
                 std::string("expect action of size ") + std::to_string(numActions()));

    omega_ = std::min(+maxOmega_, std::max(-maxOmega_, static_cast<real>(action[0])));

    axis_ = real3{0.0_r, 0.0_r, 0.0_r};

    const auto& bodies  = env_->getBodies();
    const auto& targets = env_->getTargetPositions();

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        auto u = normalized(bodies[i].r - targets[i]);
        axis_ += action[i+1] * u;
    }
        
    axis_ = normalized(axis_);
}



MagnFieldFromActionFromLocalFrame::MagnFieldFromActionFromLocalFrame(real minOmega, real maxOmega) :
    MagnFieldFromActionBase(minOmega, maxOmega)
{}

int MagnFieldFromActionFromLocalFrame::numActions() const {return 1+3;}

ActionBounds MagnFieldFromActionFromLocalFrame::getActionBounds() const
{
    return {{minOmega_, -1.0, -1.0, -1.0},
            {maxOmega_, +1.0, +1.0, +1.0}};
}

std::tuple<real3, real3, real3> MagnFieldFromActionFromLocalFrame::getFrameReference() const
{
    constexpr int NotFound = -1;
        
    const auto& bodies  = env_->getBodies();
    const auto& targets = env_->getTargetPositions();

    auto selectId = [&bodies, &targets](int start) -> int
    {
        constexpr real minDist = 5e-1_r;
        
        for (size_t i = start; i < bodies.size(); ++i)
        {
            const real3 dr = bodies[i].r - targets[i];
            const real l = length(dr);
            if (l > minDist) return i;
        }
        return NotFound;
    };
        
    const int i1 = selectId(0);

    if (i1 == NotFound)
        return {ex, ey, ez};
        
    const real3 n1 = normalized(bodies[i1].r - targets[i1]);

    const int i2 = selectId(i1 + 1);
    real3 n2;
        
    if (i2 == NotFound)
    {
        n2 = normalized(anyOrthogonal(n1));
    }
    else
    {
        n2 = normalized(bodies[i2].r - targets[i2]);
        n2 -= dot(n1, n2) * n1;
        n2 = normalized(n2);
    }

    const real3 n3 = cross(n1, n2);

    MSODE_Ensure(dot(n1, n2) < 1e-5_r,
                 "n1 and n2 are not orthogonal");
    MSODE_Ensure(std::abs(length(n3) - 1.0_r) < 1e-5_r,
                 "Bad normalization");
        
    return {n1, n2, n3};
}
    
void MagnFieldFromActionFromLocalFrame::setAction(const std::vector<double>& action)
{
    MSODE_Expect(static_cast<int>(action.size()) == numActions(),
                 std::string("expect action of size ") + std::to_string(numActions()));

    const real3 a {static_cast<real>(action[1]),
                   static_cast<real>(action[2]),
                   static_cast<real>(action[3])};
        
    omega_ = std::min(maxOmega_, std::max(minOmega_, static_cast<real>(action[0])));

    real3 n1, n2, n3;
    std::tie(n1, n2, n3) = getFrameReference();

    axis_ = a.x * n1 + a.y * n2 + a.z * n3;
    axis_ = normalized(axis_);
}

MagnFieldFromActionFromLocalPlane::MagnFieldFromActionFromLocalPlane(real minOmega, real maxOmega) :
    MagnFieldFromActionFromLocalFrame(minOmega, maxOmega)
{}

int MagnFieldFromActionFromLocalPlane::numActions() const {return 1+2;}

ActionBounds MagnFieldFromActionFromLocalPlane::getActionBounds() const
{
    return {{minOmega_, -1.0, -1.0},
            {maxOmega_, +1.0, +1.0}};
}
    
void MagnFieldFromActionFromLocalPlane::setAction(const std::vector<double>& action)
{
    MSODE_Expect(static_cast<int>(action.size()) == numActions(),
                 std::string("expect action of size ") + std::to_string(numActions()));

    const real ax = static_cast<real>(action[1]);
    const real ay = static_cast<real>(action[2]);
        
    omega_ = std::min(+maxOmega_, std::max(0.0_r, static_cast<real>(action[0])));

    real3 n1, n2, n3;
    std::tie(n1, n2, n3) = getFrameReference();

    axis_ = ax * n1 + ay * n2;
    axis_ = normalized(axis_);
}

} // namespace rl
} // namespace msode
