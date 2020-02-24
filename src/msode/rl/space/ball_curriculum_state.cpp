#include "ball_curriculum_state.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvSpaceBallCuriculumStateRW::EnvSpaceBallCuriculumStateRW(real radius, real targetRadius, real sigmaRandomWalk) :
    EnvSpaceBall(radius),
    targetRadius_(targetRadius),
    sigmaRandomWalk_(sigmaRandomWalk)
{}

std::unique_ptr<EnvSpace> EnvSpaceBallCuriculumStateRW::clone() const
{
    return std::make_unique<EnvSpaceBallCuriculumStateRW>(*this);
}

static inline real3 generateOnePositionMC(std::mt19937& gen, real3 r0, real radius, real targetRadius, real sigma)
{
    bool accepted {false};
    real3 r;
    std::normal_distribution<real> distr {0.0_r, sigma};

    while (!accepted)
    {
        const real3 step {distr(gen), distr(gen), distr(gen)};
        r = r0 + step;

        if (dot(r,r) <= radius * radius &&
            dot(r,r) > targetRadius * targetRadius)
            accepted = true;
    }
    return r;
}

std::vector<real3> EnvSpaceBallCuriculumStateRW::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    
    if (!initialized_)
    {
        constexpr real eps = 0.01_r;
        previousPositions_.resize(n);
        for (auto& p : previousPositions_)
            p = utils::generateUniformPositionShell(gen, targetRadius_, targetRadius_ * (1.0_r + eps));
        initialized_ = true;
    }

    for (int i = 0; i < n; ++i)
        positions[i] = generateOnePositionMC(gen, previousPositions_[i], radius_, targetRadius_, sigmaRandomWalk_);
        
    previousPositions_ = positions;
    return positions;
}

} // namespace rl
} // namespace msode
