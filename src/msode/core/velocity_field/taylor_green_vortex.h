// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

namespace msode
{

/// Steady Taylor-green vortex flow, see initial conditions in https://en.wikipedia.org/wiki/Taylor%E2%80%93Green_vortex
class VelocityFieldTaylorGreenVortex : public BaseVelocityField
{
public:
    /** Construct a VelocityFieldTaylorGreenVortex.
        \param [in] magnitude
        \param [in] invPeriod

        This method will fail if the parameters are not satisfying the incompressibility condition
        (see https://en.wikipedia.org/wiki/Taylor%E2%80%93Green_vortex)
    */
    VelocityFieldTaylorGreenVortex(real3 magnitude, real3 invPeriod);

    std::unique_ptr<BaseVelocityField> clone() const override;

    real3 getVelocity(real3 r, real t) const override;
    real3 getVorticity(real3 r, real t) const override;

private:
    real3 magnitude_; ///< magnitudes along the 3 directions
    real3 invPeriod_; ///< inverse wave lengths along each dimension
};

} // namespace msode
