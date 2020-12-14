// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "factory.h"

#include "none.h"
#include "constant.h"
#include "sum.h"
#include "taylor_green_vortex.h"

#include <msode/core/log.h>

#include <string>

namespace msode
{
namespace factory
{

std::unique_ptr<BaseVelocityField> createVelocityField(const Config& rootConfig, const ConfPointer& confPointer)
{
    std::unique_ptr<BaseVelocityField> vf;

    const Config& config = rootConfig.at(confPointer);

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
    else if (type == "Sum")
    {
        std::vector<std::unique_ptr<BaseVelocityField>> fields;
        auto fieldsCfg = config.at("fields");

        if (!fieldsCfg.is_array())
            msode_die("Expected an array of fields in velocity field of type 'Sum'");

        for (const auto& cfg : fieldsCfg)
            fields.push_back(createVelocityField(cfg, ConfPointer("")));

        vf = std::make_unique<VelocityFieldSum>(std::move(fields));
    }
    else
    {
        msode_die("Could not generate a VelocityField object from type '%s'",
                  type.c_str());
    }
    return vf;
}

} // namespace factory
} // namespace msode
