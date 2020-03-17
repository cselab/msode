/** dump_velocity_field
    
    Read a config file compatible with one of ``run_rl`` or ``run_rl_comp`` and dump the velocity field that is described there. 
*/

#include <msode/core/velocity_field/factory.h>
#include <msode/rl/pos_ic/factory.h>

#include <fstream>

using namespace msode;

int main(int argc, char **argv)
{
    if (argc != 3)
    {
        fprintf(stderr, "usage : %s <config.json> <filename.vtk> \n\n", argv[0]);
        return 1;
    }

    const std::string configName (argv[1]);
    const std::string outputName (argv[2]);

    std::ifstream confFile(configName);

    if (!confFile.is_open())
        msode_die("Could not read config file %s", configName.c_str());
    
    const Config config = json::parse(confFile);
    
    auto field = factory::createVelocityField(config.at("velocityField"));
    auto posIc = rl::factory::createEnvPosIC(config.at("posIc"));
    
    const real time = 0.0_r;
    const real3 start = posIc->getLowestPosition();
    const real3 end   = posIc->getHighestPosition();
    const real3 size = end - start;

    const int n = 64;
    const int3 resolution {n, n, n};
    
    field->dumpToVtkUniformGrid(outputName, resolution, start, size, time);
    
    return 0;
}
