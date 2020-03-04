#include "ball_curriculum_state_drift.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvSpaceBallCurriculumStateDriftRW::EnvSpaceBallCurriculumStateDriftRW(int maxTries, real radius,
                                                                       real targetRadius, real sigmaRandomWalk,
                                                                       std::unique_ptr<BaseVelocityField> velField, real driftTime) :
    EnvSpaceBallCurriculumStateRW(maxTries, radius, targetRadius, sigmaRandomWalk),
    velField_(std::move(velField)),
    driftTime_(driftTime)
{}

EnvSpaceBallCurriculumStateDriftRW::EnvSpaceBallCurriculumStateDriftRW(const EnvSpaceBallCurriculumStateDriftRW& other) :
    EnvSpaceBallCurriculumStateRW(other),
    velField_(other.velField_->clone()),
    driftTime_(other.driftTime_)
{}

std::unique_ptr<EnvSpace> EnvSpaceBallCurriculumStateDriftRW::clone() const
{
    return std::make_unique<EnvSpaceBallCurriculumStateDriftRW>(*this);
}

std::vector<real3> EnvSpaceBallCurriculumStateDriftRW::generateNewPositions(std::mt19937& gen, int n)
{
    _setPositionsIfNotUnitialized(gen, n);
    
    std::vector<real3> positions(n);
    
    for (int i = 0; i < n; ++i)
        positions[i] = _generateOnePositionMC(gen, _applyInverseDrift(previousPositions_[i]));
        
    previousPositions_ = positions;
    return positions;
}

real3 EnvSpaceBallCurriculumStateDriftRW::_applyInverseDrift(real3 r) const
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
