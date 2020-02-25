#include <msode/core/log.h>
#include <msode/rl/factory.h>
#include <msode/rl/helpers.h>

#include <fstream>
#include <type_traits>

inline void appMain(smarties::Communicator *const comm, int /*argc*/, char **/*argv*/)
{
    using namespace msode;
    
    // ../ because we run in ${RUNDIR}/simulation%2d_%d/
    const std::string confFileName = "../config.json";
    std::ifstream confFile(confFileName);

    if (!confFile.is_open())
        msode_die("Could not open the config file '%s'", confFileName.c_str());

    const Config config = json::parse(confFile);

    auto env = rl::createEnvironment(config);
    
    rl::setActionDims  (env.get(), comm);
    rl::setActionBounds(env.get(), comm);
    rl::setStateBounds (env.get(), comm);
    
    bool isTraining {true};
    long simId {0};

    using Status = typename std::remove_pointer<decltype(env.get())>::type::Status;
    
    while (isTraining)
    {
        auto status {Status::Running};

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
