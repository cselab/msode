#pragma once

#include "ball.h"

namespace msode {
namespace rl {

class EnvPosICBallGrowing : public EnvPosICBall
{
public:
    EnvPosICBallGrowing(real targetRadius, real maxRadius, real growStep);

    std::unique_ptr<EnvPosIC> clone() const override;

    void update(bool succesfulTry) override;
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

private:
    real targetRadius_;
    real currentRadius_;
    real growStep_;
};

} // namespace rl
} // namespace msode
