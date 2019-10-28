#include "helpers.h"

#include <random>

std::vector<real3> generateRandomPositions(int n, real3 boxLo, real3 boxHi, long seed)
{
    std::vector<real3> positions;
    positions.reserve(n);

    std::mt19937 gen(seed);
    std::uniform_real_distribution<real> distrx(boxLo.x, boxHi.x);
    std::uniform_real_distribution<real> distry(boxLo.y, boxHi.y);
    std::uniform_real_distribution<real> distrz(boxLo.z, boxHi.z);

    for (int i = 0; i < n; ++i)
    {
        real3 r;
        r.x = distrx(gen);
        r.y = distry(gen);
        r.z = distrz(gen);
        positions.push_back(r);
    }

    return positions;
}
