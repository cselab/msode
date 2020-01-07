#include "utils.h"

using namespace msode;

static inline real sq(real x) {return x*x;}

real3 generateUniformBox(std::mt19937& gen, real3 lo, real3 hi)
{
    std::uniform_real_distribution<real> uniformx(lo.x, hi.x);
    std::uniform_real_distribution<real> uniformy(lo.y, hi.y);
    std::uniform_real_distribution<real> uniformz(lo.z, hi.z);
    
    return {uniformx(gen), uniformy(gen), uniformz(gen)};
}

real3 generateUniformBall(std::mt19937& gen, real radius)
{
    std::uniform_real_distribution<real> unif(-radius, radius);
    bool accepted = false;

    real3 r;
    while (not accepted)
    {
        r.x = unif(gen);
        r.y = unif(gen);
        r.z = unif(gen);

        if (sq(r.x) + sq(r.y) + sq(r.z) < sq(radius))
            accepted = true;
    }
    return r;
}
