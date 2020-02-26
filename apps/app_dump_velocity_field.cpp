#include <msode/core/velocity_field/factory.h>
#include <msode/core/math.h>

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
    
    auto field = factory::createVelocityField(config);
    
    const real time = 0.0_r;
    const real3 L {5.0_r, 5.0_r, 5.0_r};
    const real3 start = -0.5_r * L;
    
    field->dumpToVtkUniformGrid(outputName, {32, 32, 32}, start, L, time);
    
    return 0;
}
