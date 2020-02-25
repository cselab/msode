#include "factory.h"

#include "ball.h"
#include "ball_curriculum_state.h"
#include "ball_curriculum_action.h"
#include "box.h"

namespace msode {
namespace rl {

std::unique_ptr<EnvSpace> createEnvSpace(const Config& config)
{
    std::unique_ptr<EnvSpace> es;

    const auto type = config.at("__type").get<std::string>();

    if (type == "Box")
    {
        es = std::make_unique<EnvSpaceBox>(config.at("L").get<real>());
    }
    else if (type == "Ball")
    {
        es = std::make_unique<EnvSpaceBall>(config.at("radius").get<real>());
    }
    else if (type == "BallCuriculumState")
    {
        es = std::make_unique<EnvSpaceBallCuriculumStateRW>(config.at("radius").get<real>(),
                                                            config.at("targetRadius").get<real>(),
                                                            config.at("sigma").get<real>());
    }
    else if (type == "BallCuriculumAction")
    {
        msode_die("Not implemented yet '%s'", type.c_str());
    }
    else
    {
        msode_die("Could not generate an EnvSpace object from type '%s'",
                  type.c_str());
    }
    return es;
}

} // namespace rl
} // namespace msode
