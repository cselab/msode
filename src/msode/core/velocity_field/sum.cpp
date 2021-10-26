// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "sum.h"

#include <msode/core/math.h>

namespace msode
{

VelocityFieldSum::VelocityFieldSum(std::vector<std::unique_ptr<BaseVelocityField>> fields) :
    fields_(std::move(fields))
{}

std::unique_ptr<BaseVelocityField> VelocityFieldSum::clone() const
{
    std::vector<std::unique_ptr<BaseVelocityField>> fieldClones;

    for (const auto& f : fields_)
        fieldClones.push_back(f->clone());

    return std::make_unique<VelocityFieldSum>(std::move(fieldClones));
}

real3 VelocityFieldSum::getVelocity(real3 r, real t) const
{
    real3 v {0.0_r, 0.0_r, 0.0_r};

    for (const auto& f : fields_)
        v += f->getVelocity(r, t);

    return v;
}

real3 VelocityFieldSum::getVorticity(real3 r, real t) const
{
    real3 w {0.0_r, 0.0_r, 0.0_r};

    for (const auto& f : fields_)
        w += f->getVorticity(r, t);

    return w;
}

DeformationRateTensor VelocityFieldSum::getDeformationRateTensor(real3 r, real t) const
{
    DeformationRateTensor T {0.0_r, 0.0_r, 0.0_r, 0.0_r, 0.0_r, 0.0_r};
    for (const auto& f : fields_)
    {
        const auto Tf = f->getDeformationRateTensor(r, t);
        T.xx += Tf.xx;
        T.xy += Tf.xy;
        T.xz += Tf.xz;
        T.yy += Tf.yy;
        T.yz += Tf.yz;
        T.zz += Tf.zz;
    }

    return T;
}

} // namespace msode
