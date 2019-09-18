#include "environment.h"
#include "factory.h"
#include "file_parser.h"
#include "magnetic_field_from_action.h"
#include "smarties.h"

static auto createBodies(const std::string& fileNameList)
{
    std::vector<RigidBody> bodies;
    const FileParser parser(fileNameList);

    for (auto entry : parser)
        bodies.push_back(Factory::readRigidBodyConfig(entry.second));

    return bodies;
}

static real computeMaxDistance(Box src, real3 dst)
{
    auto distFromDst = [dst] (real3 r) {return length(r-dst);};
    real d{0.0_r};
    for (auto r : src.getCorners())
        d = std::max(d, distFromDst(r));
    return d;
}

static real computeMinForwardVelocity(real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    real v = 1e9_r;
    
    for (const auto& body : bodies)
    {
        const real vmaxBody = fieldMagnitude * length(body.magnMoment) * fabs(body.propulsion.B[0]);
        v = std::min(v, vmaxBody);
    }
    return v;
}

static real computeTimeToTravel(real maxDistance, real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    return maxDistance / computeMinForwardVelocity(fieldMagnitude, bodies);
}

static real computeActionTimeScale(real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    real wmax {0._r};

    for (const auto& body : bodies)
    {
        const auto& C = body.propulsion.C;
        const real cmax = std::max(C[1], C[2]);
        const real w = length(body.magnMoment) * fieldMagnitude * cmax;
        wmax = std::max(wmax, w);
    }
    return 1.0_r / wmax;
}

static real computeMaxOmegaNoSlip(real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    real wmax {0._r};

    for (const auto& body : bodies)
    {
        const auto& C = body.propulsion.C;
        const real w = length(body.magnMoment) * fieldMagnitude * fabs(C[0]);
        wmax = std::max(wmax, w);
    }
    return wmax;
}

inline void appMain(smarties::Communicator *const comm, int argc, char **argv)
{
    // ../ because we run in ${RUNDIR}/simulation%2d_%d/
    const auto bodies = createBodies("../config/swimmers_list.cfg");
    
    const int nbodies = bodies.size();
    
    // parameters
    
    const real fieldMagnitude = 1.0_r;
    const real dt = 1e-3_r;
    const real L = 50.0_r;
    const Box box{{-L, -L, -L},
                  {+L, +L, +L}};
    const real3 target {0.0_r, 0.0_r, 0.0_r};
    const real maxDistance = computeMaxDistance(box, target);
    const real distanceThreshold = 2.0_r;

    const real endRewardK      = 5.0_r * maxDistance * nbodies;
    const real endRewardBeta   = 1.0_r / (nbodies * distanceThreshold * distanceThreshold);
    
    const real timeCoeffReward = 0.1_r * nbodies * L * computeMinForwardVelocity(fieldMagnitude, bodies);
    const real tmax = 10.0_r * computeTimeToTravel(maxDistance, fieldMagnitude, bodies);
    const real dtAction = 10.0_r * computeActionTimeScale(fieldMagnitude, bodies);
    const long nstepsPerAction = dtAction / dt;
    const long dumpEvery {1000}; // TODO
    // const long dumpEvery {100};
    const TimeParams timeParams {dt, tmax, nstepsPerAction, dumpEvery};
    const RewardParams rewardParams {timeCoeffReward, endRewardBeta, endRewardK};
    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);

    const Params params {timeParams, rewardParams, fieldMagnitude, distanceThreshold, box};

    fprintf(stderr,
            "----------------------------------------------------------\n"
            "tmax %g ; steps %ld ; max omega %g\n"
            "fieldMagnitude %g\n"
            "timeCoeffReward %g\n"
            "----------------------------------------------------------\n",
            tmax, nstepsPerAction, maxOmega,
            fieldMagnitude, timeCoeffReward);
    
    const std::vector<real3> targetPositions(nbodies, target);

    // using MagnFieldActionType = MagnFieldFromActionDirect;
    // MagnFieldActionType magnFieldAction(maxOmega);

    using MagnFieldActionType = MagnFieldFromActionFromTargets;
    MagnFieldActionType magnFieldAction(maxOmega);

    using Status = MSodeEnvironment<MagnFieldActionType>::Status;
    

    MSodeEnvironment<MagnFieldActionType> env(params, bodies, targetPositions, magnFieldAction);
    

    const int nControlVars = env.numActions();
    const int nStateVars = env.getState().size();
    comm->set_state_action_dims(nStateVars, nControlVars);

    // action bounds
    {
        const bool bounded = true;
        std::vector<double> lo, hi;
        std::tie(lo, hi) = env.getActionBounds();
        comm->set_action_scales(hi, lo, bounded);
    }

    // state bounds
    {
        std::vector<double> lo, hi;
        real3 minr {target}, maxr {target};
        auto min3 = [](real3 a, real3 b) {return real3{std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};};
        auto max3 = [](real3 a, real3 b) {return real3{std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};};

        minr = min3(minr, box.lo);
        maxr = max3(maxr, box.hi);

        for (size_t i = 0; i < bodies.size(); ++i)
        {
            // quaternions, positions
            lo.insert(lo.end(), {-1.0_r, -1.0_r, -1.0_r, -1.0_r, minr.x, minr.y, minr.z});
            hi.insert(hi.end(), {+1.0_r, +1.0_r, +1.0_r, +1.0_r, maxr.x, maxr.y, maxr.z});
        }
        
        comm->set_state_scales(hi, lo);
    }
    
    bool isTraining {true};

    while (isTraining)
    {
        auto status {Status::Running};

        env.reset(comm->getPRNG());
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
    }
}

int main(int argc, char **argv)
{
    smarties::Engine e(argc, argv);
    if( e.parse() ) return 1;
    e.run( appMain );
    return 0;
}
