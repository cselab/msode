#pragma once

#include "ball.h"

namespace msode {
namespace rl {

class MSodeEnvironment;

class EnvPosICBallCurriculumActionRW : public EnvPosICBall
{
public:
    EnvPosICBallCurriculumActionRW(int maxTries, std::unique_ptr<MSodeEnvironment>&& environment,
                                   real radius, real targetRadius, real sigmaRandomWalk);

    std::unique_ptr<EnvPosIC> clone() const override;
    
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    std::vector<double> _generateAction(std::mt19937& gen) const;
    
protected:
    const real targetRadius_;
    const real sigmaRandomWalk_;
    
    bool initialized_ {false};
    std::vector<real3> previousPositions_;

    // shared because too lazy to write clone with unique_ptr
    std::shared_ptr<MSodeEnvironment> environment_;
};

} // namespace rl
} // namespace msode
