// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "shear.h"

namespace msode
{

VelocityFieldShear::VelocityFieldShear(real G) :
    G_(G)
{}

std::unique_ptr<BaseVelocityField> VelocityFieldShear::clone() const
{
    return std::make_unique<VelocityFieldShear>(*this);
}

real3 VelocityFieldShear::getVelocity(real3 r, real /* t */) const
{
    return {r.y * G_, 0.0_r, 0.0_r};
}

real3 VelocityFieldShear::getVorticity(real3 /* r */, real /* t */) const
{
    return {0.0_r, 0.0_r, G_};
}

DeformationRateTensor VelocityFieldShear::getDeformationRateTensor(real3 /* r */, real /* t */) const
{
    return {0.0_r, 0.5_r * G_, 0.0_r, 0.0_r, 0.0_r, 0.0_r};
}

} // namespace msode
