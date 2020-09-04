// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "const.h"

#include <msode/core/math.h>

namespace msode {
namespace rl {

EnvPosICConst::EnvPosICConst(const std::vector<real3>& positions) :
    positions_(positions)
{
    MSODE_Expect(positions.size() > 0, "Need at least one position");
}

std::unique_ptr<EnvPosIC> EnvPosICConst::clone() const
{
    return std::make_unique<EnvPosICConst>(*this);
}

real3 EnvPosICConst::getLowestPosition()  const
{
    real3 rmin = positions_.front();
    for (auto r : positions_)
    {
        rmin.x = std::min(r.x, rmin.x);
        rmin.y = std::min(r.y, rmin.y);
        rmin.z = std::min(r.z, rmin.z);
    }
    return rmin;
}

real3 EnvPosICConst::getHighestPosition() const
{
    real3 rmax = positions_.front();
    for (auto r : positions_)
    {
        rmax.x = std::max(r.x, rmax.x);
        rmax.y = std::max(r.y, rmax.y);
        rmax.z = std::max(r.z, rmax.z);
    }
    return rmax;
}

std::vector<real3> EnvPosICConst::generateNewPositions(std::mt19937& /* gen */, int n)
{
    MSODE_Expect(n == static_cast<int>(positions_.size()),
                 "incompatible number of positions (required %d, have %zu",
                 n, positions_.size());

    return positions_;
}

std::vector<real3> EnvPosICConst::generateUniformPositions(std::mt19937& /* gen */, int n) const
{
    MSODE_Expect(n == static_cast<int>(positions_.size()),
                 "incompatible number of positions (required %d, have %zu",
                 n, positions_.size());

    return positions_;
}

} // namespace rl
} // namespace msode
