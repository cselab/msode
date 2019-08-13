#include "factory.h"
#include "log.h"

#include <map>
#include <string>
#include <fstream>
#include <sstream>

using DataMap = std::map<std::string, std::string>;

static DataMap readFile(const std::string& fname)
{
    DataMap dataMap;
    std::ifstream file(fname);
    Expect(file.is_open(), "could not open file for read");

    for (std::string line {}; std::getline(file, line); )
    {
        std::istringstream iss(line);
        std::string key, tmp, data;

        iss >> key;
        while (iss >> tmp) {data += " " + tmp;}
        dataMap[key] = data;
    }
    return dataMap;
}

static PropulsionMatrix::SubMatrix getSubMatrix(const std::string& str)
{
    PropulsionMatrix::SubMatrix M;
    std::istringstream iss(str);
    iss >> M[0] >> M[1] >> M[2];
    return M;
}

static real3 getReal3(const std::string& str)
{
    real3 v;
    std::istringstream iss(str);
    iss >> v.x >> v.y >> v.z;
    return v;
}

static Quaternion getQuaternion(const std::string& str)
{
    real w;
    real3 u;
    std::istringstream iss(str);
    iss >> w >> u.x >> u.y >> u.z;
    return {w, u};
}

static PropulsionMatrix getPropulsion(DataMap& dataMap)
{
    return {getSubMatrix(dataMap["A"]),
            getSubMatrix(dataMap["B"]),
            getSubMatrix(dataMap["C"])};
}

namespace Factory
{
RigidBody readRigidBodyConfig(const std::string& fname)
{
    auto data = readFile(fname);
    const PropulsionMatrix propulsion = getPropulsion(data);
    const Quaternion q = getQuaternion(data["q"]);
    const real3 r = getReal3(data["r"]);
    const real3 m = getReal3(data["m"]);

    return {q, r, m, propulsion};
}
} // namespace Factory
