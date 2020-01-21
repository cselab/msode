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

namespace msode
{

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

    // assume m is along y
    inline real stepOutFrequency(real magneticFieldMagnitude, int dir = 0) const
    {
        MSODE_Expect(std::abs(magnMoment.x) < 1e-6_r && std::abs(magnMoment.z) < 1e-6_r, "Assume m along y");
        MSODE_Expect(dir == 0 || dir == 2, "Can only compute step out frequency along x or z direction");

        const real m = length(magnMoment);
        const real C = propulsion.C[dir];
        return magneticFieldMagnitude * m * C;
    }
};

struct MagneticField
{
    MagneticField(real magnitude_, std::function<real(real)> omega_, std::function<real3(real)> rotatingDirection_) :
        magnitude(magnitude_),
        omega(omega_),
        rotatingDirection(rotatingDirection_)
    {}

    void advance(real t, real dt)
    {
        phase += omega(t) * dt;

        constexpr real twoPi = 2._r * M_PI;
        if (phase >= twoPi) phase -= twoPi;
        if (phase < 0)      phase += twoPi;
    }
    
    real3 operator()(real t) const
    {
        const real3 B {magnitude * std::cos(phase),
                       magnitude * std::sin(phase),
                       0.0_r};

        constexpr real3 originalDirection {0.0_r, 0.0_r, 1.0_r};
        const real3 currentDirection = rotatingDirection(t);

        MSODE_Ensure(length(currentDirection) > 0.0_r, "Rotating direction must be different than 0");
        
        const auto q = Quaternion::createFromVectors(originalDirection, currentDirection);
        
        return q.rotate(B);
    }

    real magnitude, phase {0};
    std::function<real(real)> omega;
    std::function<real3(real)> rotatingDirection;
};

class Simulation
{
public:

    Simulation(const std::vector<RigidBody>& initialRBs, const MagneticField& initialMF);
    ~Simulation() = default;

    Simulation(const Simulation&) = default;
    Simulation& operator=(const Simulation&) = default;

    void reset(const std::vector<RigidBody>& initialRBs, const MagneticField& initialMF);
    void activateDump(const std::string& fname, long dumpEvery);
    void run(long nsteps, real dt);

    const std::vector<RigidBody>& getBodies() const {return rigidBodies;}
    std::vector<RigidBody>& getBodies() {return rigidBodies;}
    
    const MagneticField& getField() const {return magneticField;}
    real getCurrentTime() const {return currentTime;}
    
    void advance(real dt);
    void dump();

private:
    void advanceForwardEuler(real dt);
    void advanceRK4(real dt);
    
private:
    real currentTime {0.0_r};
    long currentTimeStep {0};
    std::vector<RigidBody> rigidBodies;
    MagneticField magneticField;

    long dumpEvery {0};
    std::ofstream file {};
};


std::ostream& operator<<(std::ostream& stream, const RigidBody& b);
std::ostream& operator<<(std::ostream& stream, const PropulsionMatrix& m);

} // namespace msode
