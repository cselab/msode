#include "ball_curriculum_state.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvSpaceBallCurriculumStateRW::EnvSpaceBallCurriculumStateRW(int maxTries, real radius, real targetRadius, real sigmaRandomWalk) :
    EnvSpaceBall(maxTries, radius),
    targetRadius_(targetRadius),
    sigmaRandomWalk_(sigmaRandomWalk)
{}

std::unique_ptr<EnvSpace> EnvSpaceBallCurriculumStateRW::clone() const
{
    return std::make_unique<EnvSpaceBallCurriculumStateRW>(*this);
}

std::vector<real3> EnvSpaceBallCurriculumStateRW::generateNewPositions(std::mt19937& gen, int n)
{
    _setPositionsIfNotUnitialized(gen, n);
    
    std::vector<real3> positions(n);

    for (int i = 0; i < n; ++i)
        positions[i] = _generateOnePositionMC(gen, previousPositions_[i]);
        
    previousPositions_ = positions;
    return positions;
}

void EnvSpaceBallCurriculumStateRW::_setPositionsIfNotUnitialized(std::mt19937& gen, int n)
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

real3 EnvSpaceBallCurriculumStateRW::_generateOnePositionMC(std::mt19937& gen, real3 r0) const 
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
