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

static real computeMaxDistance(Box src, real3 dst)
{
    auto distFromDst = [dst] (real3 r) {return length(r-dst);};
    real d{0.0_r};
    for (auto r : src.getCorners())
        d = std::max(d, distFromDst(r));
    return d;
}

static real computeTimeToTravel(real maxDistance, real fieldMagnitude, const std::vector<RigidBody>& bodies)
{
    real t = 0._r;
    
    for (const auto& body : bodies)
    {
        const real vmaxBody = fieldMagnitude * length(body.magnMoment) * fabs(body.propulsion.B[0]);
        t = std::max(t, maxDistance / vmaxBody);
    }
    return t;
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
    const real bonusReward = 10.0_r;
    const real fieldMagnitude = 1.0_r;
    const real dt = 1e-3_r;
    const Box box{{-20.0_r, -10.0_r, -10.0_r},
                  {  0.0_r, +10.0_r, +10.0_r}};
    const real3 target {0.0_r, 0.0_r, 0.0_r};
    const real distanceThreshold = 0.1_r;

    const real tmax = 10 * computeTimeToTravel(computeMaxDistance(box, target), fieldMagnitude, bodies);
    const real dtAction = computeActionTimeScale(fieldMagnitude, bodies);
    const long nstepsPerAction = 0.5_r * dtAction / dt;
    const TimeParams timeParams {dt, tmax, nstepsPerAction};
    const real maxOmega = 2.0_r * computeMaxOmegaNoSlip(fieldMagnitude, bodies);

    const Params params {timeParams, maxOmega, fieldMagnitude, distanceThreshold, box};

    const std::vector<real3> targetPositions(nbodies, target);

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
