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
};

} // namespace msode
