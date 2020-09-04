// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "ball_random_walk.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvPosICBallRandomWalk::EnvPosICBallRandomWalk(real radius, real targetRadius, real sigmaRandomWalk, int curriculumTries) :
    EnvPosICBall(radius),
    targetRadius_(targetRadius),
    sigmaRandomWalk_(sigmaRandomWalk),
    curriculumTries_(curriculumTries)
{
    MSODE_Expect(curriculumTries_ == NoCurriculum ||
                 curriculumTries_ > 0,
                 "Wrong value for curriculum Tries");
}

std::unique_ptr<EnvPosIC> EnvPosICBallRandomWalk::clone() const
{
    return std::make_unique<EnvPosICBallRandomWalk>(*this);
}

void EnvPosICBallRandomWalk::update(bool succesfulTry)
{
    if (succesfulTry)
    {
        needUpdate_ = true;
        numTries_ = 0;
    }
    else
    {
        ++numTries_;

        if (numTries_ > curriculumTries_)
        {
            numTries_ = 0;
            needUpdate_ = true;
        }
        else
        {
            needUpdate_ = false;
        }
    }
}

std::vector<real3> EnvPosICBallRandomWalk::generateNewPositions(std::mt19937& gen, int n)
{
    _setPositionsIfNotUnitialized(gen, n);

    if (needUpdate_)
    {
        for (int i = 0; i < n; ++i)
            previousPositions_[i] = _generateOnePositionMC(gen, previousPositions_[i]);
    }

    return previousPositions_;
}

void EnvPosICBallRandomWalk::_setPositionsIfNotUnitialized(std::mt19937& gen, int n)
{
    if (!initialized_)
    {
        constexpr real eps = 0.01_r;
        previousPositions_.resize(n);
        for (auto& p : previousPositions_)
            p = utils::generateUniformPositionShell(gen, targetRadius_, targetRadius_ * (1.0_r + eps));
        initialized_ = true;
    }
}

real3 EnvPosICBallRandomWalk::_generateOnePositionMC(std::mt19937& gen, real3 r0) const
{
    bool accepted {false};
    real3 r;
    std::normal_distribution<real> distr {0.0_r, sigmaRandomWalk_};

    while (!accepted)
    {
        const real3 step {distr(gen), distr(gen), distr(gen)};
        r = r0 + step;

        if (dot(r,r) <= radius_ * radius_ &&
            dot(r,r) > targetRadius_ * targetRadius_)
            accepted = true;
    }
    return r;
}


} // namespace rl
} // namespace msode
