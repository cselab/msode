// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "none.h"

namespace msode
{

VelocityFieldNone::VelocityFieldNone() :
    VelocityFieldConstant({0.0_r, 0.0_r, 0.0_r})
{}

std::unique_ptr<BaseVelocityField> VelocityFieldNone::clone() const
{
    return std::make_unique<VelocityFieldNone>(*this);
}

} // namespace msode
