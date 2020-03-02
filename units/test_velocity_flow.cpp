#include <msode/core/simulation.h>
#include <msode/core/velocity_field/constant.h>
#include <msode/core/velocity_field/none.h>
#include <msode/core/velocity_field/taylor_green_vortex.h>

#include <gtest/gtest.h>
#include <random>

using namespace msode;

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

template<class VelocityField>
static void checkDivergenceFree(const VelocityField& velocityField, real3 r)
{
    constexpr real tol = 1e-6_r;
    constexpr real h = 1e-6_r;
    constexpr real invTwoH = 0.5_r / h;
    constexpr real t = 0.0_r;

    const real3 vp00 = velocityField.getVelocity(r + h * ex, t);
    const real3 v0p0 = velocityField.getVelocity(r + h * ey, t);
    const real3 v00p = velocityField.getVelocity(r + h * ez, t);

    const real3 vm00 = velocityField.getVelocity(r - h * ex, t);
    const real3 v0m0 = velocityField.getVelocity(r - h * ey, t);
    const real3 v00m = velocityField.getVelocity(r - h * ez, t);

    const real dxvx = invTwoH * (vp00.x - vm00.x);
    const real dyvy = invTwoH * (v0p0.y - v0m0.y);
    const real dzvz = invTwoH * (v00p.z - v00m.z);

    const real divergence = dxvx + dyvy + dzvz;
    
    ASSERT_LE(divergence, tol);
}

template<class VelocityField>
static void checkDivergenceFree(const VelocityField& velocityField, int nsamples = 100, long seed = 424242L)
{
    constexpr real L = 10.0_r;
    std::mt19937 gen{seed};
    std::uniform_real_distribution<real> distr(-L, L);

    for (int i = 0; i < nsamples; ++i)
    {
        const real3 r {distr(gen), distr(gen), distr(gen)};
        checkDivergenceFree(velocityField, r);
    }
}


GTEST_TEST( VELOCITY_FIELD, divergence_constant )
{
    const real3 v {-1.04_r, 2.3_r, 0.12_r};
    const VelocityFieldConstant vel{v};
    checkDivergenceFree(vel);
}

GTEST_TEST( VELOCITY_FIELD, divergence_none )
{
    const VelocityFieldNone vel{};
    checkDivergenceFree(vel);
}

GTEST_TEST( VELOCITY_FIELD, divergence_taylorGreenVortex )
{
    const real3 magn {1.0_r, 1.0_r, -2.0_r};
    const real3 invPeriod {0.3_r, 0.3_r, 0.3_r};
    const VelocityFieldTaylorGreenVortex vel{magn, invPeriod};
    checkDivergenceFree(vel);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
