#include "constant.h"

namespace msode
{

VelocityFieldConstant::VelocityFieldConstant(real3 vel) :
    vel_(vel)
{}

real3 VelocityFieldConstant::getVelocity(real3 /* r */, real /* t */) const
{
    return vel_;
}

} // namespace msode
