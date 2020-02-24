#include "ball.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvSpaceBall::EnvSpaceBall(real radius) :
    radius_(radius)
{}

std::unique_ptr<EnvSpace> EnvSpaceBall::clone() const
{
    return std::make_unique<EnvSpaceBall>(*this);
}

real3 EnvSpaceBall::getLowestPosition()  const {return {-radius_, -radius_, -radius_};}
real3 EnvSpaceBall::getHighestPosition() const {return {+radius_, +radius_, +radius_};}

real EnvSpaceBall::computeMaxDistanceToTarget() const {return radius_;}

std::vector<real3> EnvSpaceBall::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    for (auto& p : positions)
        p = utils::generateUniformPositionBall(gen, radius_);
    return positions;
}

} // namespace rl
} // namespace msode
