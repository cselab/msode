#include "environment.h"
#include "factory.h"
#include "file_parser.h"
#include "smarties.h"

static auto createBodies(const std::string& fileNameList)
{
    std::vector<RigidBody> bodies;
    const FileParser parser(fileNameList);

    for (auto entry : parser)
        bodies.push_back(Factory::readRigidBodyConfig(entry.second));

    return bodies;
}

// static real computeTmax(real maxDistance, const std::vector<RigidBody>& bodies)
// {
//     real tmax = 0._r;
    
//     for (const auto& body : bodies)
//     {
//         const real vmaxBody = 
//         tmax = std::max(tmax, );
//     }
// }

inline void appMain(smarties::Communicator *const comm, int argc, char **argv)
{
    // ../ because we run in ${RUNDIR}/simulation%2d_%d/
    const auto bodies = createBodies("../config/swimmers_list.cfg");
    
    const int nbodies = bodies.size();
    const std::vector<real3> targetPositions(nbodies, {0.0_r, 0.0_r, 0.0_r});
    
    // parameters
    const real bonusReward = 10.0_r;
    const real dt = 1e-3_r;
    const Box box{{-10.0_r, -10.0_r, -10.0_r},
                  {+10.0_r, +10.0_r, +10.0_r}};
    
    
    

    const real tmax = 20000.0_r;
    const long nstepsPerAction = 1000l;
    const TimeParams timeParams {dt, tmax, nstepsPerAction};
    const real maxOmega = 20.0_r;
    const real fieldMagnitude = 1.0_r;
    const real distanceThreshold = 0.1_r;
    const Params params {timeParams, maxOmega, fieldMagnitude, distanceThreshold, box};

    MSodeEnvironment env(params, bodies, targetPositions);

    const int nControlVars = 4; // fieldorientation (3) and frequency (1)
    const int nStateVars = env.getState().size();
    comm->set_state_action_dims(nStateVars, nControlVars);

    // action bounds
    {
        const bool bounded = true;
        const std::vector<double> upper_action_bound{+0.5 * maxOmega, +0.5, +0.5, +0.5},
                                  lower_action_bound{-0.5 * maxOmega, -0.5, -0.5, -0.5};
        comm->set_action_scales(upper_action_bound, lower_action_bound, bounded);
    }

    //OPTIONAL: set space bounds
    // std::vector<double> upper_state_bound{ 1,  1,  1,  1,  1,  1};
    // std::vector<double> lower_state_bound{-1, -1, -1, -1, -1, -1};
    // comm->set_state_scales(upper_state_bound, lower_state_bound);
    
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

            const auto& state  = env.getState();
            const auto  reward = env.getReward();

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
                comm->sendTermState(state, reward + bonusReward);
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
