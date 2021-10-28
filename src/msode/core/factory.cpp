// Copyright 2020 ETH Zurich. All Rights Reserved.
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

    RigidBody b {q, r, m, propulsion};

    if (config.contains("aspectRatio"))
        b.aspectRatio = config.at("aspectRatio").get<real>();
    if (config.contains("transDiffusion"))
        b.transDiffusion = config.at("transDiffusion").get<real>();
    if (config.contains("rotDiffusion"))
        b.rotDiffusion = config.at("rotDiffusion").get<real>();

    return b;
}

std::vector<RigidBody> readBodiesArray(const Config& config)
{
    if (!config.is_array())
        msode_die("Expected an array of bodies in config");

    std::vector<RigidBody> bodies;

    for (const auto& c : config)
        bodies.push_back(msode::factory::readRigidBodyFromConfig(c));

    return bodies;
}

} // namespace factory
} // namespace msode
