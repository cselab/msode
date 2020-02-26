#pragma once

#include "ball.h"

namespace msode {
namespace rl {

class EnvSpaceBallCurriculumStateRW : public EnvSpaceBall
{
public:
    EnvSpaceBallCurriculumStateRW(real radius, real targetRadius, real sigmaRandomWalk);

    std::unique_ptr<EnvSpace> clone() const override;
    
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    const real targetRadius_;
    const real sigmaRandomWalk_;
    
    bool initialized_ {false};
    std::vector<real3> previousPositions_;
};

} // namespace rl
} // namespace msode
