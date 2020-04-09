#include "factory.h"

#include "ball.h"
#include "ball_growing.h"
#include "ball_line.h"
#include "ball_random_walk.h"
#include "ball_random_walk_drift.h"
#include "box.h"
#include "const.h"

#include <msode/rl/factory.h>
#include <msode/core/velocity_field/factory.h>

namespace msode {
namespace rl {
namespace factory {

std::unique_ptr<EnvPosIC> createEnvPosIC(const Config& config)
{
    std::unique_ptr<EnvPosIC> es;

    const auto type = config.at("__type").get<std::string>();

    if (type == "Box")
    {
        es = std::make_unique<EnvPosICBox>(config.at("L").get<real>());
    }
    else if (type == "Ball")
    {
        es = std::make_unique<EnvPosICBall>(config.at("radius").get<real>());
    }
    else if (type == "BallLine")
    {
        es = std::make_unique<EnvPosICBallLine>(config.at("radius").get<real>(),
                                                config.at("probLine").get<real>());
    }
    else if (type == "BallRandomWalk")
    {
        es = std::make_unique<EnvPosICBallRandomWalk>(config.at("radius").get<real>(),
                                                      config.at("targetRadius").get<real>(),
                                                      config.at("sigma").get<real>(),
                                                      config.at("curriculumTries").get<int>());
    }
    else if (type == "BallRandomWalkDrift")
    {
        auto velField = msode::factory::createVelocityField(config.at("velocityField"));
        
        es = std::make_unique<EnvPosICBallRandomWalkDrift>(config.at("radius").get<real>(),
                                                           config.at("targetRadius").get<real>(),
                                                           config.at("sigma").get<real>(),
                                                           std::move(velField),
                                                           config.at("driftTime").get<real>(),
                                                           config.at("curriculumTries").get<int>());
    }
    else if (type == "BallGrowing")
    {
        es = std::make_unique<EnvPosICBallGrowing>(config.at("targetRadius").get<real>(),
                                                   config.at("radius").get<real>(),
                                                   config.at("volumeGrowStep").get<real>());
    }
    else if (type == "Const")
    {
        std::vector<real3> positions;

        auto posConf = config.at("positions");

        if (!config.is_array())
            msode_die("Const PosIC: Expected an array of real3 for variable 'positions'");

        for (auto r : posConf)
            positions.push_back(r.get<real3>());
            
        es = std::make_unique<EnvPosICConst>(positions);
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
