#include <msode/rl/environment.h>
#include <msode/rl/space/factory.h>
#include <msode/rl/field_from_action/factory.h>

#include <gtest/gtest.h>
#include <memory>

using namespace msode;
using namespace msode::rl;

constexpr real magneticFieldMagnitude = 1.0_r;
constexpr real distanceThreshold      = 2.0_r;
constexpr real domainRadius           = 50.0_r;

static PropulsionMatrix generateRandomPropulsion(std::mt19937& gen)
{
    std::uniform_real_distribution<real> unif(0.8, 1.2);
    PropulsionMatrix P;
    P.A[0]          = 0.3_r * unif(gen);
    P.A[1] = P.A[2] = 0.2_r * unif(gen);

    P.B[0] = 0.2_r;
    P.B[1] = P.B[2] = 0.0_r;

    P.C[0]          = 6.3_r * unif(gen);
    P.C[1] = P.C[2] = 1.2_r * unif(gen);
    return P;
}

static RigidBody generateRandomBody(std::mt19937& gen)
{
    const auto q = Quaternion::createIdentity();
    const real3 r {0.0_r, 0.0_r, 0.0_r};
    const real3 m {0.0_r, 2.0_r, 0.0_r};
    const auto propulsion = generateRandomPropulsion(gen);
    
    return {q, r, m, propulsion};    
}

static std::unique_ptr<MSodeEnvironment> createTestEnv(std::mt19937& gen)
{
    const RigidBody body = generateRandomBody(gen);

    const real omegaC     = body.stepOutFrequency(magneticFieldMagnitude);
    const real omegaCperp = body.stepOutFrequency(magneticFieldMagnitude, 2);

    const real orientScale = 2.0_r * M_PI / omegaCperp;
    
    TimeParams tParams;
    RewardParams rParams;
    
    tParams.dt              = 2.0_r * M_PI / omegaC / 50;
    tParams.tmax            = 500.0_r;
    tParams.nstepsPerAction = 10.0_r * orientScale / tParams.dt;
    tParams.dumpEvery       = 0;

    rParams.timeCoeff        = 0.0_r;
    rParams.terminationBonus = 0.0_r;
    
    Params params(tParams, rParams, magneticFieldMagnitude, distanceThreshold);

    auto envSpace = std::make_unique<EnvSpaceBall>(domainRadius);
    std::vector<RigidBody> initialBodies = {body};
    std::vector<real3> targetPositions(initialBodies.size(), envSpace->target);
    auto actionField = std::make_unique<FieldFromActionDirect>(0.0_r, 2.0_r * omegaC);

    return std::make_unique<MSodeEnvironment>(params, std::move(envSpace), initialBodies,
                                              targetPositions, std::move(actionField));
}

GTEST_TEST( RL_ENVIRONMENT, reward_positive_towards_target )
{
    std::mt19937 gen(4242);

    auto env = createTestEnv(gen);

    auto computeAction = [](const MSodeEnvironment *env) -> std::vector<double>
    {
        const RigidBody body = env->getBodies()[0];
        const real omega = 0.8_r * body.stepOutFrequency(magneticFieldMagnitude);
        //printf("%g %g %g\n", body.r.x, body.r.y, body.r.z);
        const real3 r = normalized(body.r);
        return {omega, r.x, r.y, r.z};
    };

    env->reset(gen);
    env->advance(computeAction(env.get()));
    
    for (int i = 0; i < 10; ++i)
    {
        env->advance(computeAction(env.get()));
        const auto reward = env->getReward();
        ASSERT_GE(reward, 0.0_r);
    }
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
