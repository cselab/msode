#pragma once

#include "ball_curriculum_state.h"

#include <msode/core/velocity_field/interface.h>

#include <memory>

namespace msode {
namespace rl {

/** \brief a EnvSpaceBallCurriculumStateRW with additional drift term from velocity field.
    This allows to take into account the advection in state space
*/
class EnvSpaceBallCurriculumStateDriftRW : public EnvSpaceBallCurriculumStateRW
{
public:
    EnvSpaceBallCurriculumStateDriftRW(int maxTries, real radius,
                                       real targetRadius, real sigmaRandomWalk,
                                       std::unique_ptr<BaseVelocityField> velField, real driftTime);

    EnvSpaceBallCurriculumStateDriftRW(const EnvSpaceBallCurriculumStateDriftRW&);

    std::unique_ptr<EnvSpace> clone() const override;
    
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

private:
    real3 _applyInverseDrift(real3 r) const;
    
private:
    std::unique_ptr<BaseVelocityField> velField_;
    real driftTime_;
};

} // namespace rl
} // namespace msode
