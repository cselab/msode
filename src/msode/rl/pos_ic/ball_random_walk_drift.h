#pragma once

#include "ball_random_walk.h"

#include <msode/core/velocity_field/interface.h>

#include <memory>

namespace msode {
namespace rl {

/** \brief a EnvPosICBallRandomWalk with additional drift term from velocity field.
    This allows to take into account the advection in state space
*/
class EnvPosICBallRandomWalkDrift : public EnvPosICBallRandomWalk
{
public:
    EnvPosICBallRandomWalkDrift(real radius, real targetRadius, real sigmaRandomWalk,
                                std::unique_ptr<BaseVelocityField> velField, real driftTime);

    EnvPosICBallRandomWalkDrift(const EnvPosICBallRandomWalkDrift&);

    std::unique_ptr<EnvPosIC> clone() const override;
    
    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

private:
    real3 _applyInverseDrift(real3 r) const;
    
private:
    std::unique_ptr<BaseVelocityField> velField_;
    real driftTime_;
};

} // namespace rl
} // namespace msode
