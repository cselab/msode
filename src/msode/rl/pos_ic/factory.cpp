#include "factory.h"

#include "ball.h"
#include "ball_growing.h"
#include "ball_random_walk.h"
#include "ball_random_walk_drift.h"
#include "box.h"

#include <msode/rl/factory.h>
#include <msode/core/velocity_field/factory.h>

namespace msode {
namespace rl {
namespace factory {

std::unique_ptr<EnvPosIC> createEnvPosIC(const Config& config)
{
    std::unique_ptr<EnvPosIC> es;

    const auto type = config.at("__type").get<std::string>();
    const int maxTries = config.at("maxTries").get<int>();

    if (type == "Box")
    {
        es = std::make_unique<EnvPosICBox>(maxTries,
                                           config.at("L").get<real>());
    }
    else if (type == "Ball")
    {
        es = std::make_unique<EnvPosICBall>(maxTries,
                                            config.at("radius").get<real>());
    }
    else if (type == "BallRandomWalk")
    {
        es = std::make_unique<EnvPosICBallRandomWalk>(maxTries,
                                                      config.at("radius").get<real>(),
                                                      config.at("targetRadius").get<real>(),
                                                      config.at("sigma").get<real>());
    }
    else if (type == "BallRandomWalkDrift")
    {
        auto velField = msode::factory::createVelocityField(config.at("velocityField"));
        
        es = std::make_unique<EnvPosICBallRandomWalkDrift>(maxTries,
                                                           config.at("radius").get<real>(),
                                                           config.at("targetRadius").get<real>(),
                                                           config.at("sigma").get<real>(),
                                                           std::move(velField),
                                                           config.at("driftTime").get<real>());
    }
    else if (type == "BallGrowing")
    {
        es = std::make_unique<EnvPosICBallGrowing>(maxTries,
                                                   config.at("radius").get<real>(),
                                                   config.at("targetRadius").get<real>(),
                                                   config.at("growStep").get<real>());
    }
    else
    {
        msode_die("Could not generate an EnvPosIC object from type '%s'",
                  type.c_str());
    }
    return es;
}

} // namespace factory
} // namespace rl
} // namespace msode
