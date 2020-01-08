#include "space.h"
#include "../utils.h"

#include <simulation.h>

using namespace msode;

EnvSpaceBox::EnvSpaceBox(real L_) :
    domain{{-L_, -L_, -L_},
           {+L_, +L_, +L_}}
{}

std::unique_ptr<EnvSpace> EnvSpaceBox::clone() const {return std::make_unique<EnvSpaceBox>(*this);}

real3 EnvSpaceBox::getLowestPosition()  const {return domain.lo;}
real3 EnvSpaceBox::getHighestPosition() const {return domain.hi;}

real EnvSpaceBox::computeMaxDistanceToTarget() const
{
    auto distFromDst = [](real3 dst, real3 r) {return length(r-dst);};
    real d{0.0_r};
    for (auto r : domain.getCorners())
        d = std::max(d, distFromDst(target, r));
    return d;
}

real3 EnvSpaceBox::generatePosition(std::mt19937& gen) const
{
    return generateUniformPositionBox(gen, domain.lo, domain.hi);
}




EnvSpaceBall::EnvSpaceBall(real R_) :
    R(R_)
{}

std::unique_ptr<EnvSpace> EnvSpaceBall::clone() const {return std::make_unique<EnvSpaceBall>(*this);}

real3 EnvSpaceBall::getLowestPosition()  const {return {-R, -R, -R};}
real3 EnvSpaceBall::getHighestPosition() const {return {+R, +R, +R};}

real EnvSpaceBall::computeMaxDistanceToTarget() const {return R;}

real3 EnvSpaceBall::generatePosition(std::mt19937& gen) const
{
    return generateUniformPositionBall(gen, R);
}

