// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "factory.h"

#include "ball.h"
#include "ball_growing.h"
#include "ball_line.h"
#include "ball_random_walk.h"
#include "ball_random_walk_drift.h"
#include "box.h"
#include "const.h"
#include "gaussian.h"
#include "time_distance.h"
#include "time_distance_curriculum.h"

#include <msode/analytic_control/helpers.h>
#include <msode/core/velocity_field/factory.h>
#include <msode/rl/factory.h>

namespace msode {
namespace rl {
namespace factory {

static auto getVelocityMatrix(const Config& rootConfig)
{
    const auto bodies = msode::factory::readBodiesArray(rootConfig.at("bodies"));
    const auto B = rootConfig.at("fieldMagnitude");
    return analytic_control::createVelocityMatrix(B, bodies);
}

std::unique_ptr<EnvPosIC> createEnvPosIC(const Config& rootConfig, const ConfPointer& confPointer)
{
    auto config = rootConfig.at(confPointer);
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
        auto velField = msode::factory::createVelocityField(config, ConfPointer("velocityField"));

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

        if (!posConf.is_array())
            msode_die("Const PosIC: Expected an array of real3 for variable 'positions'");

        for (auto r : posConf)
            positions.push_back(r.get<real3>());

        es = std::make_unique<EnvPosICConst>(positions);
    }
    else if (type == "Gaussian")
    {
        std::vector<real3> positions;

        auto posConf = config.at("positions");

        if (!posConf.is_array())
            msode_die("Const PosIC: Expected an array of real3 for variable 'positions'");

        for (auto r : posConf)
            positions.push_back(r.get<real3>());

        const auto sigma = config.at("sigma").get<real>();

        es = std::make_unique<EnvPosICGaussian>(positions, sigma);
    }
    else if (type == "TimeDistance")
    {
        auto V = getVelocityMatrix(rootConfig);
        const auto ball = config.at("ball").get<bool>();
        const auto travelTime = config.at("travelTime").get<real>();

        es = std::make_unique<EnvPosICTimeDistance>(ball, travelTime, std::move(V));
    }
    else if (type == "TimeDistanceCurriculum")
    {
        auto V = getVelocityMatrix(rootConfig);
        const auto ball = config.at("ball").get<bool>();
        const auto initTravelTime = config.at("initTravelTime").get<real>();
        const auto maxTravelTime = config.at("maxTravelTime").get<real>();
        const auto incTravelTime = config.at("incTravelTime").get<real>();

        const auto numTriesBeforeUpdate   = config.at("numTriesBeforeUpdate"  ).get<int>();
        const auto requiredSuccesfulTries = config.at("requiredSuccesfulTries").get<int>();

        es = std::make_unique<EnvPosICTimeDistanceCurriculum>(ball,
                                                              initTravelTime,
                                                              maxTravelTime,
                                                              incTravelTime,
                                                              std::move(V),
                                                              numTriesBeforeUpdate,
                                                              requiredSuccesfulTries);
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
