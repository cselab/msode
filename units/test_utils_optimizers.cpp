#include <msode/utils/optimizers/line_search.h>
#include <msode/utils/optimizers/gradient_descent.h>

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

GTEST_TEST( OPTIMIZERS, gradientDescent )
{
    constexpr real tol = 1e-5_r;

    {
        auto f = [](real3 x) { return std::pow(x.x - 1.0_r, 2) + std::pow(x.y + 2.0_r, 2) + std::pow(x.z, 2); };
        auto fder = [](real3 x) -> real3 { return {2 * (x.x - 1.0_r), 2 * (x.y + 2.0_r), 2 * x.z}; };

        real3 x {3.0_r, 3.0_r, 3.0_r};
        real fval;

        std::tie(x, fval) = utils::minimizeGradientDescent(f, fder, x, 1000, tol);
        
        ASSERT_NEAR(fval, 0.0_r, 1e-3_r);
        ASSERT_NEAR(x.x,  1.0_r, 1e-2_r);
        ASSERT_NEAR(x.y, -2.0_r, 1e-2_r);
        ASSERT_NEAR(x.z,  0.0_r, 1e-2_r);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
