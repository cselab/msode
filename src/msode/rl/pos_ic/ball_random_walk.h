#pragma once

#include "ball.h"

namespace msode {
namespace rl {

class EnvPosICBallRandomWalk : public EnvPosICBall
{
public:
    EnvPosICBallRandomWalk(int maxTries, real radius, real targetRadius, real sigmaRandomWalk);

    std::unique_ptr<EnvPosIC> clone() const override;
    
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    void _setPositionsIfNotUnitialized(std::mt19937& gen, int n);
    real3 _generateOnePositionMC(std::mt19937& gen, real3 r0) const;
    
protected:
    const real targetRadius_;
    const real sigmaRandomWalk_;
    
    bool initialized_ {false};
    std::vector<real3> previousPositions_;
};

} // namespace rl
} // namespace msode
