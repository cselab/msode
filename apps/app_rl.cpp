#include "rl/helpers.h"

inline void appMain(smarties::Communicator *const comm, int /*argc*/, char **/*argv*/)
{
    // ../ because we run in ${RUNDIR}/simulation%2d_%d/
    const auto bodies = createBodies("../config/swimmers_list.cfg");

    const real magneticFieldMagnitude = 1.0_r;

    const real L = 50.0_r; // in body lengths units
    const EnvSpace spaceInfos(L);
    
    auto env = createEnvironment(bodies, spaceInfos, magneticFieldMagnitude);
    
    setActionDims  (env, comm);
    setActionBounds(env, comm);
    setStateBounds(bodies, spaceInfos, comm);
    
    bool isTraining {true};
    long simId {0};

    using Status = typename decltype(env)::Status;
    
    while (isTraining)
    {
        auto status {Status::Running};

        env.reset(simId, comm->getPRNG());
        comm->sendInitState(env.getState());

        while (status == Status::Running) // simulation loop
        {
            const auto action = comm->recvAction();

            if (comm->terminateTraining())
                return;

            status = env.advance(action);

            const auto& state  = env.getState();
            const auto  reward = env.getReward();

            if (status == Status::Running)
                comm->sendState(state, reward);
            else
                comm->sendTermState(state, reward);
        }

        ++simId;
    }
}

int main(int argc, char **argv)
{
    smarties::Engine e(argc, argv);
    if( e.parse() ) return 1;
    e.run( appMain );
    return 0;
}
