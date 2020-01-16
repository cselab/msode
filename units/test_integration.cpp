#include <msode/utils/integrator.h>

#include <gtest/gtest.h>
#include <cmath>

using msode::real;
using namespace msode::literals;

GTEST_TEST( Integrator, linear_polynomial )
{
    auto f = [](real x) {return 2.0_r * x + 5.0_r;};
    const real Iexact = 28.0_r;
    const real Itrap  = integrateTrapez(f, -1.0_r, 3.0_r, 1000);
    ASSERT_FLOAT_EQ(Itrap, Iexact);
}

GTEST_TEST( Integrator, quadratic_polynomial )
{
    auto f = [](real x) {return x*x - 3.0_r * x + 1.0_r;};
    const real Iexact = 10.5_r;
    const real Itrap  = integrateTrapez(f, -2.0_r, 1.0_r, 1000);
    ASSERT_NEAR(Itrap, Iexact, 1e-5_r);
}

GTEST_TEST( Integrator, non_linear )
{
    const real a = -1.0_r;
    const real b =  5.0_r;
    auto f = [](real x) {return x * std::exp(x) * std::cos(x);};
    auto fIndefinitIntegral = [](real x) {return 0.5_r * std::exp(x) * ((x - 1.0_r) * std::sin(x) + x * std::cos(x));};
    const real Iexact = fIndefinitIntegral(b) - fIndefinitIntegral(a);
    const real Itrap  = integrateTrapez(f, a, b, 10000);
    ASSERT_NEAR(Itrap, Iexact, 1e-4_r);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
