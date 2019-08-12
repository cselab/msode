#pragma once

#include "quaternion.h"
#include "types.h"

#include <cmath>

struct RigidBody
{
    Quaternion q;
    real3 r, v, omega;
};

struct MagneticField
{
    real magnitude, omega;
    Quaternion q;
    
    real3 operator()(real t) const
    {
        const real wt = omega * t;
        real3 B {magnitude * std::cos(wt),
                 magnitude * std::sin(wt),
                 0.0_r};
        return q.rotate(B);
    }
};

class Simulation
{
public:

    Simulation();
    
    void advance(long nsteps, real dt);
    
private:
    RigidBody rigidBody;
    MagneticField magneticField;
};
