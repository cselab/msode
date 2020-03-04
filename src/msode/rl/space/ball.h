#pragma once

#include "interface.h"

namespace msode {
namespace rl {

class EnvSpaceBall : public EnvSpace
{
public:
    EnvSpaceBall(int maxTries, real radius);

    std::unique_ptr<EnvSpace> clone() const override;
    
    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;
    real computeMaxDistanceToTarget() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    const real radius_;
};

} // namespace rl
} // namespace msode
