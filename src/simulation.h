#pragma once

#include "quaternion.h"
#include "types.h"

#include <array>
#include <cmath>
#include <iostream>

struct PropulsionMatrix
{
    using SubMatrix = std::array<real,3>;
    SubMatrix A, B, C;
};

struct RigidBody
{
    Quaternion q;
    real3 r, magnMoment;
    PropulsionMatrix propulsion;

    real3 v {0._r, 0._r, 0._r}, omega {0._r, 0._r, 0._r}; 
};

struct MagneticField
{
    real magnitude, omega;
    real3 rotatingDirection;
    
    real3 operator()(real t) const
    {
        const real wt = omega * t;
        const real3 B {magnitude * std::cos(wt),
                       magnitude * std::sin(wt),
                       0.0_r};

        constexpr real3 originalDirection {0.0_r, 0.0_r, 1.0_r};
        const Quaternion q(originalDirection, rotatingDirection);
        
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


std::ostream& operator<<(std::ostream& stream, const RigidBody& b);
