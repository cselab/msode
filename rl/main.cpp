#include "environment.h"
#include "factory.h"
#include "smarties.h"

inline void appMain(smarties::Communicator *const comm, int argc, char **argv)
{
    std::vector<RigidBody> bodies;
    bodies.push_back(Factory::readRigidBodyConfig("/home/amlucas/msode/config/test0.cfg"));
    
    int nbodies = bodies.size();
    const int nControlVars = 4; // fieldorientation (3) and frequency (1)
    const int nStateVars = 3 + 3 * nbodies;
    comm->set_state_action_dims(nStateVars, nControlVars);

    // action bounds
    {
        bool bounded = true;
        std::vector<double> upper_action_bound{0.05, 0.1, 0.1, 0.1}, lower_action_bound{-0.05, -0.1, -0.1, -0.1};
        comm->set_action_scales(upper_action_bound, lower_action_bound, bounded);
    }

    //OPTIONAL: set space bounds
    // std::vector<double> upper_state_bound{ 1,  1,  1,  1,  1,  1};
    // std::vector<double> lower_state_bound{-1, -1, -1, -1, -1, -1};
    // comm->set_state_scales(upper_state_bound, lower_state_bound);

    real bonus = 10.0_r;
    std::vector<real3> targetPositions;

    for (int i = 0; i < nbodies; ++i)
        targetPositions.push_back({10.0_r, 0.0_r, 0.0_r}); // TODO

    Params params;
    params.nstepsPerAction = 1000l;
    params.dt = 1e-3_r;
    params.maxOmega = 14.0_r;
    params.distanceThreshold = 0.1_r;
    params.tmax = 5000.0_r;
    
    MSodeEnvironment env(params, bodies, targetPositions);
    bool isTraining {true};

    while (isTraining)
    {
        bool isRunning {true};

        env.reset(comm->getPRNG());
        comm->sendInitState(env.getState());

        while (isRunning) // simulation loop
        {
            const auto action = comm->recvAction();

            if (comm->terminateTraining())
                return;

            auto status = env.advance(action);

            const auto state  = env.getState();
            const auto reward = env.getReward();

            switch (status)
            {
            case MSodeEnvironment::Status::Running:
                comm->sendState(state, reward);
                break;
            case MSodeEnvironment::Status::MaxTimeEllapsed:
                comm->sendTermState(state, reward);
                isRunning = false;
                break;
            case MSodeEnvironment::Status::Success:
                comm->sendTermState(state, reward + bonus);
                isRunning = false;
                break;
            };
        }
    }
}

int main(int argc, char **argv)
{
    smarties::Engine e(argc, argv);
    if( e.parse() ) return 1;
    e.run( appMain );
    return 0;
}
