#include "factory.h"

#include "ball.h"
#include "ball_curriculum_state.h"
#include "ball_curriculum_state_drift.h"
#include "ball_curriculum_action.h"
#include "box.h"

#include <msode/rl/factory.h>
#include <msode/core/velocity_field/factory.h>

namespace msode {
namespace rl {
namespace factory {

std::unique_ptr<EnvSpace> createEnvSpace(const Config& config)
{
    std::unique_ptr<EnvSpace> es;

    const auto type = config.at("__type").get<std::string>();
    const int maxTries = config.at("maxTries").get<int>();

    if (type == "Box")
    {
        es = std::make_unique<EnvSpaceBox>(maxTries,
                                           config.at("L").get<real>());
    }
    else if (type == "Ball")
    {
        es = std::make_unique<EnvSpaceBall>(maxTries,
                                            config.at("radius").get<real>());
    }
    else if (type == "BallCurriculumState")
    {
        es = std::make_unique<EnvSpaceBallCurriculumStateRW>(maxTries,
                                                             config.at("radius").get<real>(),
                                                             config.at("targetRadius").get<real>(),
                                                             config.at("sigma").get<real>());
    }
    else if (type == "BallCurriculumStateDrift")
    {
        auto velField = msode::factory::createVelocityField(config.at("velocityField"));
        
        es = std::make_unique<EnvSpaceBallCurriculumStateDriftRW>(maxTries,
                                                                  config.at("radius").get<real>(),
                                                                  config.at("targetRadius").get<real>(),
                                                                  config.at("sigma").get<real>(),
                                                                  std::move(velField),
                                                                  config.at("driftTime").get<real>());
    }
    else if (type == "BallCurriculumAction")
    {
        auto environment = createEnvironment(config.at("environment"));
        es = std::make_unique<EnvSpaceBallCurriculumActionRW>(maxTries,
                                                              std::move(environment),
                                                              config.at("radius").get<real>(),
                                                              config.at("targetRadius").get<real>(),
                                                              config.at("sigma").get<real>());
    }
    else
    {
        msode_die("Could not generate an EnvSpace object from type '%s'",
                  type.c_str());
    }
    return es;
}

} // namespace factory
} // namespace rl
} // namespace msode
