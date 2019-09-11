#include "environment.h"
#include "factory.h"

#include <iostream>

inline void appTest(int argc, char **argv)
{
    std::vector<RigidBody> bodies;
    bodies.push_back(Factory::readRigidBodyConfig("/home/amlucas/msode/config/test0.cfg"));
    
    int nbodies = bodies.size();
    const int nControlVars = 4; // fieldorientation (3) and frequency (1)
    const int nStateVars = 3 + 3 * nbodies;

    int nstepsPerAction = 10000;
    real dt = 1e-3;
    std::vector<real3> targetPositions;

    for (int i = 0; i < nbodies; ++i)
        targetPositions.push_back({20.0_r, 0.0_r, 0.0_r});

    MSodeEnvironment env(nstepsPerAction, dt, bodies, targetPositions);

    // env.reset(comm->getPRNG());

    bool isRunning {true};

    while (isRunning) // simulation loop
    {
        std::vector<double> action {0.01_r, 0.02_r, 0.03_r, -0.05_r};
        auto status = env.advance(action);
        
        const auto state  = env.getState();
        const auto reward = env.getReward();

        for (const auto s : state)
            std::cout << s << " ";
        std::cout << reward << std::endl;
        
        switch (status)
        {
        case MSodeEnvironment::Status::Running:
            break;
        case MSodeEnvironment::Status::MaxTimeEllapsed:
            isRunning = false;
            break;
        }
    }
}

int main(int argc, char **argv)
{
    appTest(argc, argv);
    return 0;
}