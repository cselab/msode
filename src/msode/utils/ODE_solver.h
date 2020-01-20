#pragma once

#include <msode/core/log.h>
#include <msode/core/types.h>

namespace msode {
namespace utils {

template <class ODE>
static void forwardEulerStep(ODE& f, real dt)
{
    MSODE_Expect(dt >= 0, "time step must be positive");
    f += f.derivative() * dt;
}

} // namespace utils
} // namespace msode
