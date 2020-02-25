#include "interface.h"

#include <msode/core/log.h>
#include <msode/core/math.h>

#include <vector>
#include <fstream>

namespace msode
{

BaseVelocityField::~BaseVelocityField() = default;

void BaseVelocityField::dumpToVtkUniformGrid(const std::string& fileName, int3 dimensions, real3 start, real3 size, real t) const
{
    MSODE_Expect(dimensions.x > 0 && dimensions.y > 0 && dimensions.z > 0,
                 "grid dimensions must be positive");
    
    MSODE_Expect(size.x > 0 && size.y > 0 && size.z > 0,
                 "grid size must be non negative");

    const real3 h {size.x / dimensions.x,
                   size.y / dimensions.y,
                   size.z / dimensions.z};

    const int numElements = dimensions.x * dimensions.y * dimensions.z;

    // Compute the grid data
    
    std::vector<real3> field;
    field.reserve(numElements);

    for (int iz = 0; iz < dimensions.z; ++iz)
    {
        for (int iy = 0; iy < dimensions.y; ++iy)
        {
            for (int ix = 0; ix < dimensions.x; ++ix)
            {
                const real3 r {start.x + ix * h.x,
                               start.y + iy * h.y,
                               start.z + iz * h.z};
                
                const real3 vel = getVelocity(r, t);

                field.push_back(vel);
            }
        }
    }

    // Dump it to vtk

    std::ofstream f {fileName};

    MSODE_Ensure(f.is_open(), "Error opening " + fileName);

    f << "# vtk DataFile Version 2.0\n"
      << "Velocity field dumped from msode\n"
      << "ASCII\n";
    
    f << "DATASET STRUCTURED_POINTS\n"
      << "DIMENSIONS " << dimensions.x << ' ' << dimensions.y << ' ' << dimensions.z << '\n'
      << "ORIGIN " << start.x << ' ' << start.y << ' ' << start.z << '\n'
      << "SPACING " << h.x << ' ' << h.y << ' ' << h.z << '\n';

    f << "POINT_DATA " << numElements << "\n"
      << "VECTORS velocity float\n";

    for (auto v : field)
        f << v.x << ' ' << v.y << ' ' << v.z << '\n';
}

} // namespace msode