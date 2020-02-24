#include "box.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvSpaceBox::EnvSpaceBox(real L_) :
    domain_{{-L_, -L_, -L_},
            {+L_, +L_, +L_}}
{}

std::unique_ptr<EnvSpace> EnvSpaceBox::clone() const
{
    return std::make_unique<EnvSpaceBox>(*this);
}

real3 EnvSpaceBox::getLowestPosition()  const {return domain_.lo;}
real3 EnvSpaceBox::getHighestPosition() const {return domain_.hi;}

real EnvSpaceBox::computeMaxDistanceToTarget() const
{
    auto distFromDst = [](real3 dst, real3 r) {return length(r-dst);};
    real d{0.0_r};
    for (auto r : domain_.getCorners())
        d = std::max(d, distFromDst(target, r));
    return d;
}

std::vector<real3> EnvSpaceBox::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    for (auto& p : positions)
        p = utils::generateUniformPositionBox(gen, domain_.lo, domain_.hi); 
    return positions;
}

} // namespace rl
} // namespace msode
