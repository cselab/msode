// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include <msode/core/simulation.h>
#include <msode/core/factory.h>

#include <sstream>

namespace msode {
namespace app_utils {

std::vector<RigidBody> readBodies(const Config& config)
{
    if (!config.is_array())
        msode_die("Expected an array of bodies in config");

    std::vector<RigidBody> bodies;

    for (const auto& c : config)
        bodies.push_back(msode::factory::readRigidBodyFromConfig(c));

    return bodies;
}

// a trajectory is a sequence of TrajectoryPoints
using TrajectoryPoint = std::vector<RigidBody>;

std::vector<TrajectoryPoint> readTrajectory(const std::vector<RigidBody>& templateBodies, const std::string& fileName)
{
    std::ifstream file(fileName);

    if (!file.is_open())
        msode_die("Could not open the trajectory file '%s'", fileName.c_str());

    std::vector<TrajectoryPoint> trajectory;

    std::string line;
    while (std::getline(file, line))
    {
        std::istringstream iss(line);

        // read dummy variables (time and magnetic field)
        real t, omega;
        real3 dir;
        iss >> t >> omega >> dir.x >> dir.y >> dir.z;

        // read the bodies info
        auto bodies = templateBodies;
        for (auto& b : bodies)
        {
            iss >> b.q.w >> b.q.x >> b.q.y >> b.q.z
                >> b.r.x >> b.r.y >> b.r.z
                >> b.omega.x >> b.omega.y >> b.omega.z;
        }
        trajectory.push_back(std::move(bodies));
    }
    return trajectory;
}

} // namespace app_utils
} // namespace msode
