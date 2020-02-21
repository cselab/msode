#include "velocity_field.h"

namespace msode
{

BaseVelocityField::~BaseVelocityField() = default;

VelocityFieldConstant::VelocityFieldConstant(real3 vel) :
    vel_(vel)
{}

real3 VelocityFieldConstant::getVelocity(real3 /* r */, real /* t */) const
{
    return vel_;
}

VelocityFieldNone::VelocityFieldNone() :
    VelocityFieldConstant({0.0_r, 0.0_r, 0.0_r})
{}

} // namespace msode
