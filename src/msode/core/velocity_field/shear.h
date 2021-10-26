// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

namespace msode
{

/// shear rate along x, gradient along y: v(x,y,z) = (G * y, 0, 0)
class VelocityFieldShear : public BaseVelocityField
{
public:
    VelocityFieldShear(real G);

    std::unique_ptr<BaseVelocityField> clone() const override;

    real3 getVelocity(real3 r, real t) const override;
    real3 getVorticity(real3 r, real t) const override;
    DeformationRateTensor getDeformationRateTensor(real3 r, real t) const override;

private:
    const real G_; ///< velocity everywhere in space and time
};

} // namespace msode
