#include "ball_growing.h"

#include <msode/core/log.h>
#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvPosICBallGrowing::EnvPosICBallGrowing(int maxTries, real targetRadius, real maxRadius, real growStep) :
    EnvPosICBall(maxTries, maxRadius),
    targetRadius_(targetRadius),
    currentRadius_(targetRadius),
    growStep_(growStep)
{
    MSODE_Expect(targetRadius_ < radius_, "target radius must be smaller than the max radius");
}

std::unique_ptr<EnvPosIC> EnvPosICBallGrowing::clone() const
{
    return std::make_unique<EnvPosICBallGrowing>(*this);
}

void EnvPosICBallGrowing::update(bool succesfulTry)
{
    if (succesfulTry)
    {
        currentRadius_ = std::min(radius_, std::max(targetRadius_, currentRadius_ + growStep_));
    }
}

std::vector<real3> EnvPosICBallGrowing::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    for (auto& p : positions)
        p = utils::generateUniformPositionShell(gen, targetRadius_, currentRadius_);
    return positions;
}

} // namespace rl
} // namespace msode
