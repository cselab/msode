#include <msode/utils/ODE_solver.h>

#include <gtest/gtest.h>
#include <cmath>

using msode::real;
using namespace msode::literals;

constexpr real ax = 1.0_r;
constexpr real ay = 2.0_r;

struct ODE_Test
{
    real x;
    real y;

    ODE_Test derivative() const {return {ax * x, ay * y};}

    ODE_Test& operator+=(const ODE_Test& other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }
};

ODE_Test operator*(const ODE_Test& f, real a)
{
    return {a * f.x, a * f.y};
}


GTEST_TEST( ODE_solver, forward_euler_exp )
{
    const real dt = 1e-5_r;
    const real tend = 1.0_r;
    const int nsteps = static_cast<int>(tend / dt);

    const real x0 = 1.0_r;
    const real y0 = 0.5_r;
    ODE_Test ode {x0, y0};

    for (int i = 0; i < nsteps; ++i)
        msode::utils::forwardEulerStep(ode, dt);
    
    ASSERT_NEAR(ode.x, x0 * std::exp(ax * tend), 1e-3_r);
    ASSERT_NEAR(ode.y, y0 * std::exp(ay * tend), 1e-3_r);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
