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
} // namespace Factory
} // namespace msode
