// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "gaussian.h"

#include <msode/core/math.h>

namespace msode {
namespace rl {

EnvPosICGaussian::EnvPosICGaussian(std::vector<real3> positions, real sigma) :
    positions_(std::move(positions)),
    sigma_(sigma),
    maxDist_(3 * sigma_)
{
    MSODE_Expect(positions_.size() > 0, "Need at least one position");
    MSODE_Expect(sigma_ >= 0, "sigma must be non negative");
}

std::unique_ptr<EnvPosIC> EnvPosICGaussian::clone() const
{
    return std::make_unique<EnvPosICGaussian>(*this);
}

real3 EnvPosICGaussian::getLowestPosition()  const
{
    real3 rmin = positions_.front();
    for (auto r : positions_)
    {
        rmin.x = std::min(r.x, rmin.x);
        rmin.y = std::min(r.y, rmin.y);
        rmin.z = std::min(r.z, rmin.z);
    }

    rmin.x -= maxDist_;
    rmin.y -= maxDist_;
    rmin.z -= maxDist_;

    return rmin;
}

real3 EnvPosICGaussian::getHighestPosition() const
{
    real3 rmax = positions_.front();
    for (auto r : positions_)
    {
        rmax.x = std::max(r.x, rmax.x);
        rmax.y = std::max(r.y, rmax.y);
        rmax.z = std::max(r.z, rmax.z);
    }

    rmax.x += maxDist_;
    rmax.y += maxDist_;
    rmax.z += maxDist_;

    return rmax;
}

std::vector<real3> EnvPosICGaussian::generateNewPositions(std::mt19937& gen, int n)
{
    return generateUniformPositions(gen, n);
}

std::vector<real3> EnvPosICGaussian::generateUniformPositions(std::mt19937& gen, int n) const
{
    MSODE_Expect(n == static_cast<int>(positions_.size()),
                 "incompatible number of positions (required %d, have %zu",
                 n, positions_.size());

    auto positions = positions_;

    for (auto& pos : positions)
        pos += _randomDisplacement(gen);

    return positions;
}

real3 EnvPosICGaussian::_randomDisplacement(std::mt19937& gen) const
{
    real3 r;
    std::normal_distribution<real> d{0.0_r, sigma_};

    do
    {
        r.x = d(gen);
        r.y = d(gen);
        r.z = d(gen);
    } while(length(r) >= maxDist_);

    return r;
}

} // namespace rl
} // namespace msode
