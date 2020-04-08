#include "factory.h"

#include "weighted_targets.h"
#include "action_change.h"
#include "direct3.h"
#include "direct.h"
#include "local_frame.h"

namespace msode {
namespace rl {
namespace factory {

std::unique_ptr<FieldFromAction> createFieldFromAction(const Config& config)
{
    std::unique_ptr<FieldFromAction> fa;

    const auto type = config.at("__type").get<std::string>();
    
    if (type == "Direct")
    {
        fa = std::make_unique<FieldFromActionDirect>(config.at("minOmega").get<real>(),
                                                     config.at("maxOmega").get<real>());
    }
    else if (type == "Direct3")
    {
        fa = std::make_unique<FieldFromActionDirect3>(config.at("maxOmega").get<real>());
    }
    else if (type == "Change")
    {
        fa = std::make_unique<FieldFromActionChange>(config.at("minOmega").get<real>(),
                                                     config.at("maxOmega").get<real>(),
                                                     config.at("actionDt").get<real>());
    }
    else if (type == "LocalFrame")
    {
        fa = std::make_unique<FieldFromActionFromLocalFrame>(config.at("minOmega").get<real>(),
                                                             config.at("maxOmega").get<real>());
    }
    else if (type == "WeightedTargets")
    {
        fa = std::make_unique<FieldFromActionFromTargets>(config.at("maxOmega").get<real>());
    }
    else
    {
        msode_die("Could not generate a FieldFromAction object from type '%s'",
                  type.c_str());
    }
    return fa;
}

} // namespace factory
} // namespace rl
} // namespace msode
