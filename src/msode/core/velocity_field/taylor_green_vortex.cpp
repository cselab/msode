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

real3 VelocityFieldTaylorGreenVortex::getVelocity(real3 r, real /* t */) const
{
    return
        std::cos(invPeriod_.x * r.x) *
        std::sin(invPeriod_.y * r.y) *
        std::sin(invPeriod_.z * r.z) *
        magnitude_;
}

} // namespace msode
