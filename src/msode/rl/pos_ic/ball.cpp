#include "ball.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvPosICBall::EnvPosICBall(real radius) :
    radius_(radius)
{}

std::unique_ptr<EnvPosIC> EnvPosICBall::clone() const
{
    return std::make_unique<EnvPosICBall>(*this);
}

real3 EnvPosICBall::getLowestPosition()  const {return {-radius_, -radius_, -radius_};}
real3 EnvPosICBall::getHighestPosition() const {return {+radius_, +radius_, +radius_};}

real EnvPosICBall::computeMaxDistanceToTarget() const {return radius_;}

std::vector<real3> EnvPosICBall::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    for (auto& p : positions)
        p = utils::generateUniformPositionBall(gen, radius_);
    return positions;
}

} // namespace rl
} // namespace msode
