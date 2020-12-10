// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

#include <vector>

namespace msode {
namespace rl {

/** Positions sample on a gaussian in 3D.
    Can only be within a distance of 3 sigma.
 */
class EnvPosICGaussian : public EnvPosIC
{
public:
    EnvPosICGaussian(std::vector<real3> positions, real sigma);

    std::unique_ptr<EnvPosIC> clone() const override;

    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;
    std::vector<real3> generateUniformPositions(std::mt19937& gen, int n) const override;

private:
    /// \return A random variable centered around zero, gaussian distributed with max distance.
    real3 _randomDisplacement(std::mt19937& gen) const;

private:
    std::vector<real3> positions_;
    real sigma_;
    real maxDist_;
};

} // namespace rl
} // namespace msode
