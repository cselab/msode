#include "local_frame.h"

#include <msode/rl/environment.h>

namespace msode {
namespace rl {

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

FieldFromActionFromLocalFrame::FieldFromActionFromLocalFrame(real minOmega, real maxOmega) :
    FieldFromAction(minOmega, maxOmega)
{}

int FieldFromActionFromLocalFrame::numActions() const {return 1+3;}

ActionBounds FieldFromActionFromLocalFrame::getActionBounds() const
{
    return {{minOmega_, -1.0, -1.0, -1.0},
            {maxOmega_, +1.0, +1.0, +1.0}};
}

std::tuple<real3, real3, real3> FieldFromActionFromLocalFrame::getFrameReference() const
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
    
void FieldFromActionFromLocalFrame::setAction(const std::vector<double>& action)
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

} // namespace rl
} // namespace msode
