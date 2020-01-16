#include <msode/rl/space.h>

#include <gtest/gtest.h>
#include <cmath>

using namespace msode;

GTEST_TEST( RL_SPACE, box_samples_are_inside )
{
    std::mt19937 gen(4242);
    const real L {25.0_r};

    const int nPos = 16;
    const long nSamples = 1000;

    rl::EnvSpaceBox space(L);
    
    for (int i = 0; i < nSamples; ++i)
    {
        const auto positions = space.generateNewPositions(gen, nPos);

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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
