// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "constant.h"

namespace msode
{

/// Velocity field with zero velocity
class VelocityFieldNone : public VelocityFieldConstant
{
public:
    /// construct a VelocityFieldNone
    VelocityFieldNone();

    std::unique_ptr<BaseVelocityField> clone() const override;
};

} // namespace msode
