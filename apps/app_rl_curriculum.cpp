#include <msode/rl/helpers.h>

#include <type_traits>

inline void appMain(smarties::Communicator *const comm, int /*argc*/, char **/*argv*/)
{
    using namespace msode;
    
    // ../ because we run in ${RUNDIR}/simulation%2d_%d/
    const auto bodies = msode::rl::createBodies("../config/swimmers_list.cfg");

    const real magneticFieldMagnitude = 1.0_r;

    const real distanceThreshold = 0.1_r;
    const real R = 50.0_r; // domain radius, in body lengths units
    const real sigmaRW = 0.5_r;
    const rl::EnvSpaceBallCuriculumStateRW spaceInfos(R, distanceThreshold, sigmaRW);
    
    // auto env = createEnvironment(bodies, &spaceInfos, magneticFieldMagnitude, distanceThreshold);
    auto env = rl::createEnvironmentCurriculum(bodies, magneticFieldMagnitude, distanceThreshold, R, sigmaRW);
    
    rl::setActionDims  (env.get(), comm);
    rl::setActionBounds(env.get(), comm);
    rl::setStateBounds(bodies, &spaceInfos, comm);
    
    bool isTraining {true};
    long simId {0};

    using Status = typename std::remove_pointer<decltype(env.get())>::type::Status;

    // Status prevStatus {Status::Running};
    
    while (isTraining)
    {
        Status status {Status::Running};

        //const bool usePreviousIC = (prevStatus == Status::MaxTimeEllapsed);
        //env->reset(simId, comm->getPRNG(), usePreviousIC);
        env->reset(comm->getPRNG(), simId);
        comm->sendInitState(env->getState());

        while (status == Status::Running) // simulation loop
        {
            const auto action = comm->recvAction();

            if (comm->terminateTraining())
                return;

            status = env->advance(action);

            const auto& state  = env->getState();
            const auto  reward = env->getReward();

            if (status == Status::Running)
                comm->sendState(state, reward);
            else
                comm->sendTermState(state, reward);
        }

        // prevStatus = status;
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
