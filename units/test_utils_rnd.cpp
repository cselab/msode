#include <msode/utils/rnd.h>

#include <gtest/gtest.h>
#include <cmath>

using namespace msode;

GTEST_TEST( rnd, box )
{
    std::mt19937 gen(4242);
    const real3 lo {-1.0_r, 2.0_r, -3.0_r};
    const real3 hi { 1.0_r, 10.0_r, -1.0_r};

    const long nSamples = 1000;

    for (int i = 0; i < nSamples; ++i)
    {
        const auto r = utils::generateUniformPositionBox(gen, lo, hi);
        ASSERT_LE(r.x, hi.x);
        ASSERT_LE(r.y, hi.y);
        ASSERT_LE(r.z, hi.z);

        ASSERT_GE(r.x, lo.x);
        ASSERT_GE(r.y, lo.y);
        ASSERT_GE(r.z, lo.z);
    }
}

GTEST_TEST( rnd, ball )
{
    std::mt19937 gen(4242);
    const real R {5.0_r};

    const long nSamples = 1000;

    for (int i = 0; i < nSamples; ++i)
    {
        const auto r = utils::generateUniformPositionBall(gen, R);
        ASSERT_LE(dot(r,r), R*R);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
