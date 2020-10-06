// Copyright 2020 ETH Zurich. All Rights Reserved.
/** run_rl_comp

    Use smarties to find optimal policy for the problem stated in `run_ac` and compare to the `run_ac` results for the same setup.
 */

#include "rl_helpers.h"

#include <msode/analytic_control/apply_strategy.h>
#include <msode/analytic_control/optimal_path.h>
#include <msode/core/log.h>
#include <msode/core/velocity_field/factory.h>
#include <msode/rl/factory.h>

#include <fstream>
#include <iomanip>
#include <type_traits>

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
    const std::string confFileName = "../config.json";
    std::ifstream confFile(confFileName);

    if (!confFile.is_open())
        msode_die("Could not open the config file '%s'", confFileName.c_str());

    const Config config = json::parse(confFile);

    const real magneticFieldMagnitude = config.at("fieldMagnitude").get<real>();

    auto env = rl::factory::createEnvironment(config, ConfPointer(""));

    const int dumpEvery = 1000;

    const analytic_control::MatrixReal V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, env->getBodies());
    const analytic_control::MatrixReal U = V.inverse();

    setActionDims  (env.get(), comm);
    setActionBounds(env.get(), comm);
    setStateBounds (env.get(), comm);

    bool isTraining {true};
    long simId {0};

    using Status = typename std::remove_pointer<decltype(env.get())>::type::Status;

    Status previousStatus {Status::Success};

    while (isTraining)
    {
        auto status {Status::Running};

        const bool succesfulPreviousTry = previousStatus == Status::Success;
        env->reset(comm->getPRNG(), simId, succesfulPreviousTry);
        comm->sendInitState(env->getState());

        const real initDistance = computeMinDistance(env->getBodies());
        const real tAC =  [&]()
        {
            const auto envBodies = env->getBodies();
            auto velocityField = factory::createVelocityField(config, ConfPointer("/velocityField"));
            return analytic_control::simulateOptimalPath(magneticFieldMagnitude, envBodies, extractPositions(envBodies),
                                                         std::move(velocityField), U, generateACfname(simId), dumpEvery);
        }();

        while (status == Status::Running) // simulation loop
        {
            const auto action = comm->recvAction();

            if (comm->terminateTraining())
                return;

            status = env->advance(action);

            const auto& state  = env->getState();
            const auto  reward = env->getReward();

            switch (status)
            {
            case Status::Running:
                comm->sendState(state, reward);
                break;
            case Status::Success:
                comm->sendTermState(state, reward);
                break;
            case Status::MaxTimeEllapsed:
                comm->sendLastState(state, reward);
                break;
            }

            if (status != Status::Running)
            {
                const auto envBodies = env->getBodies();
                const real tRL = env->getSimulationTime();
                dumpComparisonInfos(std::cout, simId, tAC, tRL, envBodies, initDistance);
            }
        }

        previousStatus = status;
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
