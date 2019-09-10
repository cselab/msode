#pragma once

#include "quaternion.h"
#include "types.h"

#include <array>
#include <cmath>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

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
    std::function<real3(real)> rotatingDirection;
    
    real3 operator()(real t) const
    {
        const real wt = omega * t;
        const real3 B {magnitude * std::cos(wt),
                       magnitude * std::sin(wt),
                       0.0_r};

        constexpr real3 originalDirection {0.0_r, 0.0_r, 1.0_r};
        const Quaternion q(originalDirection, rotatingDirection(t));
        
        return q.rotate(B);
    }
};

class Simulation
{
public:

    Simulation(const std::vector<RigidBody>& initialRBs, const MagneticField& initialMF);
    ~Simulation() = default;

    Simulation(const Simulation&) = default;
    Simulation& operator=(const Simulation&) = default;

    void activateDump(const std::string& fname, long dumpEvery);
    void run(long nsteps, real dt);
    
private:
    void advance(real dt);
    void dump();
    
private:
    real t {0.0_r};
    long timeStep {0};
    std::vector<RigidBody> rigidBodies;
    MagneticField magneticField;

    long dumpEvery {0};
    std::ofstream file {};
};


std::ostream& operator<<(std::ostream& stream, const RigidBody& b);
std::ostream& operator<<(std::ostream& stream, const PropulsionMatrix& m);
