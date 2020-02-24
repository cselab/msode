#pragma once

#include "interface.h"

namespace msode
{
/// Velocity field constant in time and space
class VelocityFieldConstant : public BaseVelocityField
{
public:
    /// construct a VelocityFieldConstant with constant velocity \p vel
    VelocityFieldConstant(real3 vel);

    real3 getVelocity(real3 r, real t) const override;
    
private:
    const real3 vel_; ///< velocity everywhere in space and time
};

} // namespace msode
