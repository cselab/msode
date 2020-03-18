#include "box.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvPosICBox::EnvPosICBox(real L_) :
    domain_{{-L_, -L_, -L_},
            {+L_, +L_, +L_}}
{}

std::unique_ptr<EnvPosIC> EnvPosICBox::clone() const
{
    return std::make_unique<EnvPosICBox>(*this);
}

real3 EnvPosICBox::getLowestPosition()  const {return domain_.lo;}
real3 EnvPosICBox::getHighestPosition() const {return domain_.hi;}

real EnvPosICBox::computeMaxDistanceToTarget() const
{
    auto distFromDst = [](real3 dst, real3 r) {return length(r-dst);};
    real d{0.0_r};
    for (auto r : domain_.getCorners())
        d = std::max(d, distFromDst(target, r));
    return d;
}

std::vector<real3> EnvPosICBox::generateNewPositions(std::mt19937& gen, int n)
{
    return generateUniformPositions(gen, n);
}

std::vector<real3> EnvPosICBox::generateUniformPositions(std::mt19937& gen, int n) const
{
    std::vector<real3> positions(n);
    for (auto& p : positions)
        p = utils::generateUniformPositionBox(gen, domain_.lo, domain_.hi); 
    return positions;
}


} // namespace rl
} // namespace msode
