#include "factory.h"
#include "file_parser.h"
#include "log.h"

#include <map>
#include <string>
#include <fstream>
#include <sstream>

namespace msode
{

static PropulsionMatrix getPropulsion(const FileParser& parser)
{
    return {parser.getSubMatrix("A"),
            parser.getSubMatrix("B"),
            parser.getSubMatrix("C")};
}

namespace Factory
{
RigidBody readRigidBodyConfig(const std::string& fname)
{
    const FileParser parser(fname);
    const PropulsionMatrix propulsion = getPropulsion(parser);
    const Quaternion q = parser.getQuaternion("q");
    const real3 r = parser.getReal3("r");
    const real3 m = parser.getReal3("m");

    return {q, r, m, propulsion};
}


RigidBody readRigidBodyFromConfig(const Config& config)
{
    const PropulsionMatrix propulsion = config.at("propulsion").get<PropulsionMatrix>();
    const Quaternion q = config.at("quaternion").get<Quaternion>();
    const real3 r = config.at("position").get<real3>();
    const real3 m = config.at("moment").get<real3>();

    return {q, r, m, propulsion};
}
} // namespace Factory
} // namespace msode
