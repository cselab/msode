#include <msode/utils/optimizers/line_search.h>

#include <gtest/gtest.h>
#include <cmath>

using namespace msode;

GTEST_TEST( OPTIMIZERS, lineSearch )
{
    constexpr real tol = 1e-5_r;

    {
        auto f1 = [](real x) { return std::cos(2 * M_PI * x); };

        ASSERT_NEAR(utils::lineSearchGoldenSection(f1, 1.0_r, tol), 0.5_r, tol);
    }

    {
        const real a = 0.9_r;
        const real b = -5.1_r;
        const real c = -0.3_r;
        auto f2 = [a, b, c](real x) {return c + x * (b + x * a); };
        
        ASSERT_NEAR(utils::lineSearchGoldenSection(f2, -b/a + 3.435_r, tol), -0.5_r * b / a, tol);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
