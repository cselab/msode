// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "taylor_green_vortex.h"

#include <msode/core/log.h>
#include <msode/core/math.h>

namespace msode
{

VelocityFieldTaylorGreenVortex::VelocityFieldTaylorGreenVortex(real3 magnitude, real3 invPeriod) :
    magnitude_(magnitude),
    invPeriod_(invPeriod)
{
    MSODE_Expect(dot(magnitude_, invPeriod_) < 1e-6_r,
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

DeformationRateTensor VelocityFieldTaylorGreenVortex::getDeformationRateTensor(real3 r, real /* t */) const
{
    const real cx = std::cos(invPeriod_.x * r.x);
    const real sx = std::sin(invPeriod_.x * r.x);

    const real cy = std::cos(invPeriod_.y * r.y);
    const real sy = std::sin(invPeriod_.y * r.y);

    const real cz = std::cos(invPeriod_.z * r.z);
    const real sz = std::sin(invPeriod_.z * r.z);

    DeformationRateTensor T;

    T.xx = -magnitude_.x * invPeriod_.x * sx * sy * sz;
    T.yy = -magnitude_.y * invPeriod_.y * sx * sy * sz;
    T.zz = -magnitude_.z * invPeriod_.z * sx * sy * sz;

    T.xy = 0.5_r * (magnitude_.x * invPeriod_.y + magnitude_.y * invPeriod_.x) * cx * cy * sz;
    T.xz = 0.5_r * (magnitude_.x * invPeriod_.z + magnitude_.z * invPeriod_.x) * cx * sy * cz;
    T.yz = 0.5_r * (magnitude_.y * invPeriod_.z + magnitude_.z * invPeriod_.y) * sx * cy * cz;

    return T;
}

} // namespace msode
