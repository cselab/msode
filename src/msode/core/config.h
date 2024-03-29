// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include <nlohmann/json.hpp>

#include "quaternion.h"
#include "simulation.h"
#include "types.h"

namespace msode
{

using json = nlohmann::json;
using Config = json;
using ConfPointer = json::json_pointer;

void to_json(json& j, const PropulsionMatrix& p);
void from_json(const json& j, PropulsionMatrix& p);

void to_json(json& j, const Quaternion& q);
void from_json(const json& j, Quaternion& q);

void to_json(json& j, const real3& v);
void from_json(const json& j, real3& v);

} // namespace msode
