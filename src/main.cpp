#include "simulation.h"

int main(int argc, char **argv)
{
    const real magneticMomentMagnitude {2.0_r};

    const real magneticFieldMagnitude {1.0_r};
    const real omegaField {0.5_r};

    auto rotatingDirection = [&](real t)
    {
        const real omega = 0.2_r;
        const real wt_2  = 0.5_r * t * omega;
        constexpr real3 original {1._r, 0._r, 0._r};
        constexpr real3 axis {0._r, 0._r, 1._r};
        const Quaternion q{std::cos(wt_2), std::sin(wt_2) * axis};
        return q.rotate(original);
    };
    
    const Quaternion initialOrientation {0._r, 1._r, 0._r, 0._r};
    const real3 initialPosition{0._r, 0._r, 0._r};
    const real3 magneticMoment{0.0_r, magneticMomentMagnitude, 0._r};
    const PropulsionMatrix propulsion {{1._r, 1._r, 1._r},
                                       {0.1_r, 0._r, 0._r},
                                       {0.7_r, 0.5_r, 0.5_r}};
    
    RigidBody rigid {initialOrientation,
                     initialPosition,
                     magneticMoment,
                     propulsion};

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};
    
    Simulation simulation(rigid, magneticField);

    const real tEnd = 2000.0_r;
    const real tDump = 0.1_r;
    const real dt {0.001_r};
    const long nsteps = tEnd / dt;

    simulation.activateDump("out.txt", tDump / dt);
    simulation.run(nsteps, dt);
    
    return 0;
}
