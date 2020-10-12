// Copyright 2020 ETH Zurich. All Rights Reserved.
/** dump_velocity_field

    Read a config file compatible with one of ``run_rl`` or ``run_rl_comp`` and dump the velocity field that is described there.
*/

#include <msode/core/velocity_field/factory.h>

#include <fstream>

using namespace msode;

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        fprintf(stderr, "usage : %s <config.json> <filename.vtk> <L> \n\n", argv[0]);
        return 1;
    }

    const std::string configName (argv[1]);
    const std::string outputName (argv[2]);
    const real L = static_cast<real>( std::atof(argv[3]) );

    std::ifstream confFile(configName);

    if (!confFile.is_open())
        msode_die("Could not read config file %s", configName.c_str());

    const Config config = json::parse(confFile);

    auto field = factory::createVelocityField(config, ConfPointer("/velocityField"));

    const real time = 0.0_r;
    const real3 start {-L/2, -L/2, -L/2};
    const real3 end   {L/2, L/2, L/2};
    const real3 size = end - start;

    const int n = 64;
    const int3 resolution {n, n, n};

    field->dumpToVtkUniformGrid(outputName, resolution, start, size, time);

    return 0;
}
