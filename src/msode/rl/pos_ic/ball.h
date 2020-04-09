#pragma once

#include "interface.h"

namespace msode {
namespace rl {

class EnvPosICBall : public EnvPosIC
{
public:
    EnvPosICBall(real radius);

    std::unique_ptr<EnvPosIC> clone() const override;
    
    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;
    std::vector<real3> generateUniformPositions(std::mt19937& gen, int n) const override;

protected:
    const real radius_;
};

} // namespace rl
} // namespace msode
