#include "factory.h"
#include "file_parser.h"
#include "log.h"

#include <string>
#include <fstream>

namespace msode
{

namespace factory
{
RigidBody readRigidBodyConfigFromFile(const std::string& fname)
{
    std::ifstream f(fname);

    if (!f.is_open())
        msode_die("Could not open the file '%s'", fname.c_str());

    const Config config = json::parse(f);

    return readRigidBodyFromConfig(config);
}


RigidBody readRigidBodyFromConfig(const Config& config)
{
    const PropulsionMatrix propulsion = config.at("propulsion").get<PropulsionMatrix>();
    const Quaternion q = config.at("quaternion").get<Quaternion>();
    const real3 r = config.at("position").get<real3>();
    const real3 m = config.at("moment").get<real3>();

    return {q, r, m, propulsion};
}
} // namespace factory
} // namespace msode
