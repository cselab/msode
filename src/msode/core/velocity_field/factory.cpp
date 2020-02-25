#include "factory.h"

#include "none.h"
#include "constant.h"
#include "taylor_green_vortex.h"

#include <msode/core/log.h>

#include <string>

namespace msode
{

std::unique_ptr<BaseVelocityField> createVelocityField(const Config& config)
{
    std::unique_ptr<BaseVelocityField> vf;

    const auto type = config.at("__type").get<std::string>();

    if (type == "None")
    {
        vf = std::make_unique<VelocityFieldNone>();
    }
    else if (type == "Constant")
    {
        vf = std::make_unique<VelocityFieldConstant>(config.at("vel").get<real3>());
    }
    else if (type == "FieldTaylorGreenVortex")
    {
        vf = std::make_unique<VelocityFieldTaylorGreenVortex>(config.at("magnitude").get<real3>(),
                                                              config.at("invPeriod").get<real3>());
    }
    else
    {
        msode_die("Could not generate a VelocityField object from type '%s'",
                  type.c_str());
    }
    return vf;
}

} // namespace msode
