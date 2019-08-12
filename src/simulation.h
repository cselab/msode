#pragma once

#include "quaternion.h"
#include "types.h"

#include <array>
#include <cmath>

struct PropulsionMatrix
{
    using SubMatrix = std::array<real,3>;
    SubMatrix A, B, C;
};

struct RigidBody
{
    Quaternion q;
    real3 r, v, omega, magnMoment;
    PropulsionMatrix propulsion;
};

struct MagneticField
{
    real magnitude, omega;
    // real3 rotatingDirection;
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

    Simulation(const RigidBody& initialRB, const MagneticField& initialMF);
    ~Simulation() = default;

    Simulation(const Simulation&) = default;
    Simulation& operator=(const Simulation&) = default;
    
    void advance(long nsteps, real dt);
    
private:
    real t {0.0_r};
    RigidBody rigidBody;
    MagneticField magneticField;
};
