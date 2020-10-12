// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "ball_growing.h"

#include <msode/core/log.h>
#include <msode/core/math.h>
#include <msode/utils/rnd.h>

#include <cmath>

namespace msode {
namespace rl {

EnvPosICBallGrowing::EnvPosICBallGrowing(real targetRadius, real maxRadius, real volumeGrowStep) :
    EnvPosICBall(maxRadius),
    targetRadius_(targetRadius),
    currentRadius_(targetRadius),
    volumeGrowStep_(volumeGrowStep)
{
    MSODE_Expect(targetRadius_ < radius_, "target radius must be smaller than the max radius");
    MSODE_Expect(volumeGrowStep_ > 0.0_r, "volumeGrowStep must be strictly positive");
}

std::unique_ptr<EnvPosIC> EnvPosICBallGrowing::clone() const
{
    return std::make_unique<EnvPosICBallGrowing>(*this);
}

void EnvPosICBallGrowing::update(bool successfulTry)
{
    if (successfulTry)
    {
        const real V0 = 4.0_r * M_PI / 3.0_r * currentRadius_ * currentRadius_ * currentRadius_;
        currentRadius_ = std::pow((V0 + volumeGrowStep_) * 3.0_r / (4.0_r * M_PI), 1.0_r / 3.0_r);
        currentRadius_ = std::min(radius_, std::max(targetRadius_, currentRadius_));
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
