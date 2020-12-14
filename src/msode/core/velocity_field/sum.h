// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

#include <vector>

namespace msode
{

/** The sum of other velocity fields.
 */
class VelocityFieldSum: public BaseVelocityField
{
public:
    VelocityFieldSum(std::vector<std::unique_ptr<BaseVelocityField>> fields);

    std::unique_ptr<BaseVelocityField> clone() const override;

    real3 getVelocity(real3 r, real t) const override;
    real3 getVorticity(real3 r, real t) const override;

private:
    std::vector<std::unique_ptr<BaseVelocityField>> fields_;
};

} // namespace msode
