#include "weighted_targets.h"

#include <msode/rl/environment.h>

namespace msode {
namespace rl {

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

FieldFromActionFromTargets::FieldFromActionFromTargets(real maxOmega) :
    FieldFromAction(0.0, maxOmega)
{}

int FieldFromActionFromTargets::numActions() const {return 1 + env_->getBodies().size();}

ActionBounds FieldFromActionFromTargets::getActionBounds() const
{
    std::vector<double> lo{-maxOmega_}, hi{+maxOmega_};

    for (size_t i = 0; i < env_->getBodies().size(); ++i)
    {
        lo.push_back(0.0_r);
        hi.push_back(1.0_r);
    }
        
    return {std::move(lo), std::move(hi)};
}

std::tuple<real3, real3, real3> FieldFromActionFromTargets::getFrameReference() const
{
    return {ex, ey, ez};
}
    
void FieldFromActionFromTargets::setAction(const std::vector<double>& action)
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

} // namespace rl
} // namespace msode
