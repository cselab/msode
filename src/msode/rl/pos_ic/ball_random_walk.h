// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "ball.h"

namespace msode {
namespace rl {

class EnvPosICBallRandomWalk : public EnvPosICBall
{
public:
    static constexpr int NoCurriculum = 0;

    EnvPosICBallRandomWalk(real radius, real targetRadius, real sigmaRandomWalk, int curriculumTries = NoCurriculum);

    std::unique_ptr<EnvPosIC> clone() const override;

    void update(bool successfulTry) override;
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

protected:
    void _setPositionsIfNotUnitialized(std::mt19937& gen, int n);
    real3 _generateOnePositionMC(std::mt19937& gen, real3 r0) const;

protected:
    const real targetRadius_;
    const real sigmaRandomWalk_;

    bool needUpdate_ {true};
    bool initialized_ {false};
    std::vector<real3> previousPositions_;

private:
    int numTries_ {0};
    const int curriculumTries_;
};

} // namespace rl
} // namespace msode
