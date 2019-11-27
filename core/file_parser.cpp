#include "file_parser.h"
#include "log.h"

#include <fstream>
#include <sstream>

namespace msode
{

static auto readFile(const std::string& fname)
{
    FileParser::DataMap dataMap;
    std::ifstream file(fname);
    MSODE_Expect(file.is_open(), "could not open file '" + fname + "' for read");

    for (std::string line {}; std::getline(file, line); )
    {
        std::istringstream iss(line);
        std::string key, tmp, data;

        iss >> key;
        iss >> data;
        while (iss >> tmp) {data += " " + tmp;}
        dataMap[key] = data;
    }
    return dataMap;
}


FileParser::FileParser(const std::string& filename) :
    data(readFile(filename))
{}


std::string FileParser::getStr(const std::string& key) const
{
    auto it = data.find(key);
    if (it != data.end())
        return it->second;
    MSODE_Ensure(it != data.end(), "could not find key");
    return std::string();
}

real3 FileParser::getReal3(const std::string& key) const
{
    const auto valStr = getStr(key);
    real3 v;
    std::istringstream iss(valStr);
    iss >> v.x >> v.y >> v.z;
    return v;
}

Quaternion FileParser::getQuaternion(const std::string& key) const
{
    const auto valStr = getStr(key);
    real w;
    real3 u;
    std::istringstream iss(valStr);
    iss >> w >> u.x >> u.y >> u.z;
    return Quaternion::createFromComponents(w, u);
}

PropulsionMatrix::SubMatrix FileParser::getSubMatrix(const std::string& key) const
{
    const auto valStr = getStr(key);
    PropulsionMatrix::SubMatrix M;
    std::istringstream iss(valStr);
    iss >> M[0] >> M[1] >> M[2];
    return M;
}
         
} // namespace msode
