/** apps/step_out_frequencies
    
    Compute  the step out frequencies of all ABFs in the given config
 */

#include <msode/core/simulation.h>
#include <msode/core/factory.h>
#include <msode/utils/mean_vel.h>

#include <iostream>

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

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "usage : %s <config.json>\n\n", argv[0]);
        return 1;
    }

    std::ifstream confFile(argv[1]);

    if (!confFile.is_open())
        msode_die("Could not open the config file '%s'", argv[1]);

    const Config config = json::parse(confFile);

    const real magneticFieldMagnitude = config.at("fieldMagnitude");
    const auto bodies = readBodies(config.at("bodies"));

    for (auto body : bodies)
    {
        std::cout << body.stepOutFrequency(magneticFieldMagnitude) << std::endl;
    }
    
    return 0;
}
