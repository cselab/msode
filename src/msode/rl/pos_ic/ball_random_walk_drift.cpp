#include "ball_random_walk_drift.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvPosICBallRandomWalkDrift::EnvPosICBallRandomWalkDrift(int maxTries, real radius,
                                                         real targetRadius, real sigmaRandomWalk,
                                                         std::unique_ptr<BaseVelocityField> velField, real driftTime) :
    EnvPosICBallRandomWalk(maxTries, radius, targetRadius, sigmaRandomWalk),
    velField_(std::move(velField)),
    driftTime_(driftTime)
{}

EnvPosICBallRandomWalkDrift::EnvPosICBallRandomWalkDrift(const EnvPosICBallRandomWalkDrift& other) :
    EnvPosICBallRandomWalk(other),
    velField_(other.velField_->clone()),
    driftTime_(other.driftTime_)
{}

std::unique_ptr<EnvPosIC> EnvPosICBallRandomWalkDrift::clone() const
{
    return std::make_unique<EnvPosICBallRandomWalkDrift>(*this);
}

std::vector<real3> EnvPosICBallRandomWalkDrift::generateNewPositions(std::mt19937& gen, int n)
{
    _setPositionsIfNotUnitialized(gen, n);
    
    std::vector<real3> positions(n);
    
    for (int i = 0; i < n; ++i)
        positions[i] = _generateOnePositionMC(gen, _applyInverseDrift(previousPositions_[i]));
        
    previousPositions_ = positions;
    return positions;
}

real3 EnvPosICBallRandomWalkDrift::_applyInverseDrift(real3 r) const
{
    constexpr int nsteps = 100; // TODO
    const real dt = driftTime_ / nsteps;

    for (int i = 0; i < nsteps; ++i)
    {
        constexpr real t = 0; // TODO
        r -= dt * velField_->getVelocity(r, t);
    }
    return r;
}

} // namespace rl
} // namespace msode
