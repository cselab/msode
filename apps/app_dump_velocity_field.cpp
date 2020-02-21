#include <msode/core/velocity_field.h>
#include <msode/core/math.h>

#include <iostream>

using namespace msode;

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        fprintf(stderr, "usage : %s <filename.vtk> \n\n", argv[0]);
        return 1;
    }

    const std::string fileName (argv[1]);
    const real time = 0.0_r;
    const real3 L {5.0_r, 5.0_r, 5.0_r};
    const real3 start = -0.5_r * L;
    
    const real3 magnitude {L.x, L.y, -2.0_r * L.z};
    const real3 invPeriod {M_PI / L.x, M_PI / L.y, M_PI / L.z};
    
    VelocityFieldTaylorGreenVortex field(magnitude, invPeriod);

    field.dumpToVtkUniformGrid(fileName, {32, 32, 32}, start, L, time);
    
    return 0;
}
