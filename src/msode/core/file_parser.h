// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "quaternion.h"
#include "simulation.h"
#include "types.h"

#include <map>
#include <string>

namespace msode
{
class FileParser
{
public:
    using DataMap = std::map<std::string, std::string>;

    FileParser(const std::string& filename);

    auto begin() const {return data_.begin();}
    auto end() const {return data_.end();}

    std::string getStr(const std::string& key) const;
    real3 getReal3(const std::string& key) const;
    Quaternion getQuaternion(const std::string& key) const;
    PropulsionMatrix::SubMatrix getSubMatrix(const std::string& key) const;

private:
    const DataMap data_;
};
} // namespace msode
