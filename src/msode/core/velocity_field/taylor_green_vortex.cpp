#include "taylor_green_vortex.h"

#include <msode/core/log.h>
#include <msode/core/math.h>

namespace msode
{

VelocityFieldTaylorGreenVortex::VelocityFieldTaylorGreenVortex(real3 magnitude, real3 invPeriod) :
    magnitude_(magnitude),
    invPeriod_(invPeriod)
{
    const real incompressibilityError = dot(magnitude_, invPeriod_);
    MSODE_Expect(incompressibilityError < 1e-6_r,
                 "The parameters must satisfy the incompressibility condition");
}

std::unique_ptr<BaseVelocityField> VelocityFieldTaylorGreenVortex::clone() const
{
    return std::make_unique<VelocityFieldTaylorGreenVortex>(*this);
}

real3 VelocityFieldTaylorGreenVortex::getVelocity(real3 r, real /* t */) const
{
    const real cx = std::cos(invPeriod_.x * r.x);
    const real sx = std::sin(invPeriod_.x * r.x);
    
    const real cy = std::cos(invPeriod_.y * r.y);
    const real sy = std::sin(invPeriod_.y * r.y);
    
    const real cz = std::cos(invPeriod_.z * r.z);
    const real sz = std::sin(invPeriod_.z * r.z);
    
    return {magnitude_.x * cx * sy * sz,
            magnitude_.y * sx * cy * sz,
            magnitude_.z * sx * sy * cz};
}

real3 VelocityFieldTaylorGreenVortex::getVorticity(real3 r, real /* t */) const
{
    const real cx = std::cos(invPeriod_.x * r.x);
    const real sx = std::sin(invPeriod_.x * r.x);
    
    const real cy = std::cos(invPeriod_.y * r.y);
    const real sy = std::sin(invPeriod_.y * r.y);
    
    const real cz = std::cos(invPeriod_.z * r.z);
    const real sz = std::sin(invPeriod_.z * r.z);

    const real3 w = cross(invPeriod_, magnitude_);
    
    return {sx * cy * cz * w.x,
            cx * sy * cz * w.y,
            cx * cy * sz * w.z};
}

} // namespace msode
