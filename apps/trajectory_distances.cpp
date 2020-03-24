/** trajectory_distances.cpp
    
    Compute the given TargetDistance at all points of a given trajectory of ABFs.
 */

#include <msode/core/factory.h>
#include <msode/core/simulation.h>
#include <msode/rl/target_distances/factory.h>
#include <msode/utils/mean_vel.h>

#include <iostream>
#include <sstream>

using namespace msode;

static std::vector<RigidBody> readBodies(const Config& config)
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

static std::vector<TrajectoryPoint> readTrajectory(const std::vector<RigidBody>& templateBodies, const std::string& fileName)
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

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        fprintf(stderr, "usage : %s <config.json> <trajectory.dat>\n\n", argv[0]);
        return 1;
    }

    std::ifstream confFile(argv[1]);

    if (!confFile.is_open())
        msode_die("Could not open the config file '%s'", argv[1]);

    const Config config = json::parse(confFile);

    const real magneticFieldMagnitude = config.at("fieldMagnitude").get<real>();
    const auto bodies = readBodies(config.at("bodies"));
    const auto targetDistance = rl::factory::createTargetDistance(config.at("targetDistance"));

    const auto trajectory = readTrajectory(bodies, argv[2]);

    for (const auto& tp : trajectory)
    {
        const real d = targetDistance->compute(tp);
        printf("%g\n", d);
    }
    
    return 0;
}
