#include "rl/helpers.h"
#include "analytic_control/optimal_path.h"
#include "analytic_control/apply_strategy.h"



static inline std::string generateACfname(long simId)
{
    std::ostringstream ss;
    ss << std::setw(6) << std::setfill('0') << simId;
    return "ac_trajectories_" + ss.str() + ".txt";
}

static inline std::vector<real3> extractPositions(const std::vector<RigidBody>& bodies)
{
    std::vector<real3> positions;
    positions.reserve(bodies.size());

    for (const auto& b : bodies)
        positions.push_back(b.r);

    return positions;
}

static void dumpComparisonInfos(std::ostream& stream, int simId, real timeAC, real timeRL, const std::vector<RigidBody>& bodiesRL)
{
    auto getDistance = [](const RigidBody& b) {return length(b.r);};
    
    real maxDistance = 0;
    for (auto b : bodiesRL)
        maxDistance = std::max(maxDistance, getDistance(b));
    
    stream << simId << " " << timeAC << " " << timeRL << " " << maxDistance << std::endl;
}


inline void appMain(smarties::Communicator *const comm, int /*argc*/, char **/*argv*/)
{
    // ../ because we run in ${RUNDIR}/simulation%2d_%d/
    const auto bodies = createBodies("../config/swimmers_list.cfg");

    const real magneticFieldMagnitude = 1.0_r;

    const real L = 50.0_r; // in body lengths units
    const EnvSpace spaceInfos(L);
    const int dumpEvery = 1000;
    
    auto env = createEnvironment(bodies, spaceInfos, magneticFieldMagnitude);

    const MatrixReal V = createVelocityMatrix(magneticFieldMagnitude, bodies);
    const MatrixReal U = V.inverse();
    
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

        const real tAC = simulateOptimalPath(magneticFieldMagnitude,
                                             env.getBodies(),
                                             extractPositions(env.getBodies()), U,
                                             generateACfname(simId), dumpEvery);
        
        while (status == Status::Running) // simulation loop
        {
            const auto action = comm->recvAction();

            if (comm->terminateTraining())
                return;

            status = env.advance(action);

            const auto& state  = env.getState();
            const auto  reward = env.getReward();

            if (status == Status::Running)
            {
                comm->sendState(state, reward);
            }
            else
            {
                comm->sendTermState(state, reward);
                const real tRL = env.getSimulationTime();
                dumpComparisonInfos(std::cout, simId, tAC, tRL, env.getBodies());
            }
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
