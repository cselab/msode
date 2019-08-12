#pragma once

#include "quaternion.h"
#include "types.h"


struct RigidBody
{
    Quaternion q;
    real3 r, v, omega;
};

class Simulation
{
public:

private:
    RigidBody rigidBody;
};
