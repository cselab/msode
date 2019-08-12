#include "simulation.h"

int main(int argc, char **argv)
{
    const real magneticMomentMagnitude {1.0_r};

    const real magneticFieldMagnitude {1.0_r};
    const real omegaField {0.5_r};
    const real3 rotatingDirection {1._r, 0._r, 0._r}; // TODO
    
    const Quaternion initialOrientation {0._r, 1._r, 0._r, 0._r};
    const real3 initialPosition{0._r, 0._r, 0._r};
    const real3 magneticMoment{magneticMomentMagnitude, 0._r, 0._r};
    const PropulsionMatrix propulsion {{1._r, 1._r, 1._r}, {1._r, 0._r, 0._r}, {1._r, 1._r, 1._r}};
    
    RigidBody rigid {initialOrientation,
                     initialPosition,
                     magneticMoment,
                     propulsion};

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};
    
    Simulation simulation(rigid, magneticField);

    const real dt {0.1_r};
    const long nsteps {10};
    simulation.advance(nsteps, dt);
    
    return 0;
}
