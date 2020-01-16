#include <msode/analytic_control/optimal_path.h>
#include <msode/analytic_control/apply_strategy.h>
#include <msode/rl/helpers.h>

#include <type_traits>

#include <iomanip>

using namespace msode;

static inline std::string generateACfname(long simId)
{
    std::ostringstream ss;
    ss << std::setw(6) << std::setfill('0') << simId;
    return "ac_trajectories_" + ss.str() + ".dat";
}

static inline std::vector<real3> extractPositions(const std::vector<RigidBody>& bodies)
{
    std::vector<real3> positions;
    positions.reserve(bodies.size());

    for (const auto& b : bodies)
        positions.push_back(b.r);

    return positions;
}

static inline real getDistance(const RigidBody& b) {return length(b.r);}

static real computeMinDistance(const std::vector<RigidBody>& bodies)
{
    real minDistance = std::numeric_limits<real>::max();
    for (auto b : bodies)
        minDistance = std::min(minDistance, getDistance(b));
    return minDistance;
}

static real computeMaxDistance(const std::vector<RigidBody>& bodies)
{
    real maxDistance = 0;
    for (auto b : bodies)
        maxDistance = std::max(maxDistance, getDistance(b));
    return maxDistance;
}

static inline void dumpComparisonInfos(std::ostream& stream, int simId, real timeAC, real timeRL, const std::vector<RigidBody>& bodiesRL, real initDistance)
{
    const real maxDistance = computeMaxDistance(bodiesRL);
    stream << simId << " " << timeAC << " " << timeRL << " " << maxDistance << " " << initDistance << std::endl;
}


inline void appMain(smarties::Communicator *const comm, int /*argc*/, char **/*argv*/)
{
    // ../ because we run in ${RUNDIR}/simulation%2d_%d/
    const auto bodies = rl::createBodies("../config/swimmers_list.cfg");

    const real magneticFieldMagnitude = 1.0_r;

    const real L = 50.0_r; // in body lengths units
    rl::EnvSpaceBox spaceInfos(L);
    const int dumpEvery = 1000;

    auto env = rl::createEnvironment(bodies, &spaceInfos, magneticFieldMagnitude);

    const analytic_control::MatrixReal V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);
    const analytic_control::MatrixReal U = V.inverse();
    
    rl::setActionDims  (env.get(), comm);
    rl::setActionBounds(env.get(), comm);
    rl::setStateBounds(bodies, &spaceInfos, comm);
    
    bool isTraining {true};
    long simId {0};

    using Status = typename std::remove_pointer<decltype(env.get())>::type::Status;
    
    while (isTraining)
    {
        auto status {Status::Running};

        env->reset(comm->getPRNG(), simId);
        comm->sendInitState(env->getState());

        const real initDistance = computeMinDistance(env->getBodies());
        const real tAC =  [&]()
        {
            const auto envBodies = env->getBodies();
            return analytic_control::simulateOptimalPath(magneticFieldMagnitude, envBodies, extractPositions(envBodies), U, generateACfname(simId), dumpEvery);
        }();

        while (status == Status::Running) // simulation loop
        {
            const auto action = comm->recvAction();

            if (comm->terminateTraining())
                return;

            status = env->advance(action);

            const auto& state  = env->getState();
            const auto  reward = env->getReward();

            if (status == Status::Running)
            {
                comm->sendState(state, reward);
            }
            else
            {
                comm->sendTermState(state, reward);

                const auto envBodies = env->getBodies();
                const real tRL = env->getSimulationTime();
                dumpComparisonInfos(std::cout, simId, tAC, tRL, envBodies, initDistance);
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
