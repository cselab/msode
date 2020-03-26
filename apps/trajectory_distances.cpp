/** trajectory_distances.cpp
    
    Compute the given TargetDistance at all points of a given trajectory of ABFs.
 */

#include "utils.h"

#include <msode/core/factory.h>
#include <msode/core/simulation.h>
#include <msode/rl/target_distances/factory.h>

#include <iostream>

using namespace msode;


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
    const auto bodies = app_utils::readBodies(config.at("bodies"));
    const auto targetDistance = rl::factory::createTargetDistance(config.at("targetDistance"));

    const auto trajectory = app_utils::readTrajectory(bodies, argv[2]);

    for (const auto& tp : trajectory)
    {
        const real d = targetDistance->compute(tp);
        printf("%g\n", d);
    }
    
    return 0;
}
