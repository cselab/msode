// Copyright 2020 ETH Zurich. All Rights Reserved.
/** trajectory_sdf.cpp

    Compute the distance field of a set of ABF trajectories along with the velocity and vorticity fields
 */

#include "utils.h"

#include <msode/core/factory.h>
#include <msode/core/simulation.h>
#include <msode/core/velocity_field/factory.h>
#include <msode/rl/pos_ic/factory.h>

#include <iostream>
#include <vector>

using namespace msode;


using Segments = std::vector<real3>;
static std::vector<Segments> extractSegments(const std::vector<app_utils::TrajectoryPoint>& traj)
{
    std::vector<Segments> allSegments;
    const int n = traj.front().size();

    allSegments.resize(n);

    for (auto& segments : allSegments)
        segments.reserve(traj.size());

    for (const auto& tp : traj)
    {
        for (size_t i = 0; i < n; ++i)
            allSegments[i].push_back(tp[i].r);
    }
    return allSegments;
}

static inline real distanceToSegment(real3 r, real3 a, real3 b)
{
    const auto ar = r - a;
    const auto ab = b - a;
    real alpha = dot(ab, ar) / dot(ab,ab);
    alpha = std::min(1.0_r, std::max(0.0_r, alpha));
    const auto p = a + alpha * ab;

    return length(r - p);
}

static real distanceToSegments(real3 r, const Segments& segments)
{
    MSODE_Expect(segments.size() >= 2, "Need at least 2 positions to make a set of segments");

    real distance = length(r - segments.front());

    for (size_t i = 0; i < segments.size() - 1; ++i)
    {
        const real di = distanceToSegment(r, segments[i], segments[i+1]);
        distance = std::min(distance, di);
    }

    return distance;
}

static real distanceToAllSegments(real3 r, const std::vector<Segments>& allSegments)
{
    real distance = std::numeric_limits<real>::max();
    for (const auto& segments : allSegments)
        distance = std::min(distance, distanceToSegments(r, segments));
    return distance;
}

int main(int argc, char **argv)
{
    if (argc != 7)
    {
        fprintf(stderr, "usage : %s <config.json> <trajectory.dat> <field_and_sdf.vtk> <L> <n> <l>\n\n", argv[0]);
        return 1;
    }

    std::ifstream confFile(argv[1]);

    if (!confFile.is_open())
        msode_die("Could not open the config file '%s'", argv[1]);

    const Config config = json::parse(confFile);

    const real magneticFieldMagnitude = config.at("fieldMagnitude").get<real>();
    const auto bodies = app_utils::readBodies(config.at("bodies"));
    auto field = factory::createVelocityField(config, ConfPointer("/velocityField"));

    const auto trajectory = app_utils::readTrajectory(bodies, argv[2]);
    const auto allSegments = extractSegments(trajectory);

    const real L = static_cast<real>( std::atof(argv[4]) );
    const int n = std::atoi(argv[5]);
    const real l = static_cast<real>( std::atof(argv[6]) );

    // values on the grid

    const int3 res {n, n, n};

    const real3 start {-L/2, -L/2, -L/2};
    const real3 end   {L/2, L/2, L/2};
    const real3 size = end - start;

    msode::Filter filter = [&](real3 r){return distanceToAllSegments(r, allSegments) < l;};

    // dump to vtk

    const char *outFileName = argv[3];
    std::ofstream f {outFileName};

    MSODE_Ensure(f.is_open(), "Error opening file '%s'", outFileName);

    const real time {0.0_r};
    field->dumpToVtkUniformGrid(f, res, start, size, time, filter);

    return 0;
}
