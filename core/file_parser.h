#pragma once

#include "quaternion.h"
#include "simulation.h"
#include "types.h"

#include <map>
#include <string>

class FileParser
{
public:
    using DataMap = std::map<std::string, std::string>;

    FileParser(const std::string& filename);

    std::string getStr(const std::string& key) const;
    real3 getReal3(const std::string& key) const;
    Quaternion getQuaternion(const std::string& key) const;
    PropulsionMatrix::SubMatrix getSubMatrix(const std::string& key) const;

private:
    const DataMap data;
};
