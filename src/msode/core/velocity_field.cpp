#include "velocity_field.h"

#include <msode/core/log.h>
#include <msode/core/math.h>

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


VelocityFieldTaylorGreenVortex::VelocityFieldTaylorGreenVortex(real3 magnitude, real3 invPeriod) :
    magnitude_(magnitude),
    invPeriod_(invPeriod)
{
    const real incompressibilityError = dot(magnitude_, invPeriod_);
    MSODE_Ensure(incompressibilityError < 1e-6_r,
                 "The parameters must satisfy the incompressibility condition");
}

real3 VelocityFieldTaylorGreenVortex::getVelocity(real3 r, real /* t */) const
{
    return
        std::cos(invPeriod_.x * r.x) *
        std::sin(invPeriod_.y * r.y) *
        std::sin(invPeriod_.z * r.z) *
        magnitude_;
}

} // namespace msode
