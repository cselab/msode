#include <msode/core/math.h>
#include <msode/core/velocity_field/constant.h>
#include <msode/rl/pos_ic/factory.h>
#include <msode/rl/pos_ic/box.h>
#include <msode/rl/pos_ic/ball.h>
#include <msode/rl/pos_ic/ball_curriculum_state.h>
#include <msode/rl/pos_ic/ball_curriculum_state_drift.h>

#include <gtest/gtest.h>
#include <cmath>
#include <memory>

using namespace msode;

GTEST_TEST( RL_POS_IC, box_samples_are_inside )
{
    std::mt19937 gen(4242);
    const real L {25.0_r};

    const int nPos = 16;
    const long nSamples = 1000;

    constexpr int maxTries = 1;
    rl::EnvPosICBox posIc(maxTries, L);
    
    for (int i = 0; i < nSamples; ++i)
    {
        const auto positions = posIc.generateNewPositions(gen, nPos);

        for (auto r : positions)
        {
            ASSERT_LE(r.x, L);
            ASSERT_LE(r.y, L);
            ASSERT_LE(r.z, L);

            ASSERT_GE(r.x, -L);
            ASSERT_GE(r.y, -L);
            ASSERT_GE(r.z, -L);
        }
    }
}

GTEST_TEST( RL_POS_IC, ball_samples_are_inside )
{
    std::mt19937 gen(4242);
    const real R {25.0_r};

    const int nPos = 16;
    const long nSamples = 1000;

    constexpr int maxTries = 1;
    rl::EnvPosICBall posIc(maxTries, R);
    
    for (int i = 0; i < nSamples; ++i)
    {
        const auto positions = posIc.generateNewPositions(gen, nPos);

        for (auto r : positions)
        {
            ASSERT_LE(dot(r,r), R*R);
        }
    }
}

GTEST_TEST( RL_POS_IC, ball_curriculum_state_samples_are_inside )
{
    std::mt19937 gen(4242);
    const real R {25.0_r};
    const real targetRadius {2.0_r};
    const real sigmaRW = targetRadius * 0.5_r;

    const int nPos = 16;
    const long nSamples = 1000;

    constexpr int maxTries = 1;
    rl::EnvPosICBallCurriculumStateRW posIc(maxTries, R, targetRadius, sigmaRW);
    
    for (int i = 0; i < nSamples; ++i)
    {
        const auto positions = posIc.generateNewPositions(gen, nPos);

        for (auto r : positions)
        {
            ASSERT_GE(dot(r,r), targetRadius*targetRadius);
            ASSERT_LE(dot(r,r), R*R);
        }
    }
}

GTEST_TEST( RL_POS_IC, ball_curriculum_state_drift_correct_drift )
{
    std::mt19937 gen(4242);
    const real R {25.0_r};
    const real targetRadius {2.0_r};
    const real sigmaRW = 0.0_r; // no rw in this case

    const real3 vel {3.0_r, 4.0_r, 5.0_r};
    const real driftTime {1.2_r};

    // otherwise we might end up in an infinite loop
    ASSERT_GE(length(vel) * driftTime, 2*targetRadius);
    ASSERT_LE(length(vel) * driftTime, R - targetRadius);

    const int nPos = 16;
    const long nSamples = 1000;

    constexpr int maxTries = 1;
    rl::EnvPosICBallCurriculumStateDriftRW posIc(maxTries, R, targetRadius, sigmaRW,
                                                 std::make_unique<VelocityFieldConstant>(vel),
                                                 driftTime);
    
    const auto initPos = posIc.generateNewPositions(gen, nPos);
    const auto nextPos = posIc.generateNewPositions(gen, nPos);

    for (size_t i = 0; i < initPos.size(); ++i)
    {
        constexpr real tol = 1e-6_r;
        
        const real3 refDrift = -driftTime * vel;
        const real3 drift = nextPos[i] - initPos[i];

        ASSERT_NEAR(drift.x, refDrift.x, tol);
        ASSERT_NEAR(drift.y, refDrift.y, tol);
        ASSERT_NEAR(drift.z, refDrift.z, tol);
    }
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
