#include <msode/core/simulation.h>
#include <msode/core/velocity_field/constant.h>
#include <msode/core/velocity_field/none.h>
#include <msode/core/velocity_field/sum.h>
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
    std::mt19937 gen(seed);
    std::uniform_real_distribution<real> distr(-L, L);

    for (int i = 0; i < nsamples; ++i)
    {
        const real3 r {distr(gen), distr(gen), distr(gen)};
        checkDivergenceFree(velocityField, r);
    }
}


template<class VelocityField>
static void checkVorticity(const VelocityField& velocityField, real3 r)
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

    const real3 dxv = invTwoH * (vp00 - vm00);
    const real3 dyv = invTwoH * (v0p0 - v0m0);
    const real3 dzv = invTwoH * (v00p - v00m);

    // finite differences
    const real3 wfd {dyv.z - dzv.y,
                     dzv.x - dxv.z,
                     dxv.y - dyv.x};


    const real3 wref = velocityField.getVorticity(r, t);

    ASSERT_NEAR(wfd.x, wref.x, tol);
    ASSERT_NEAR(wfd.y, wref.y, tol);
    ASSERT_NEAR(wfd.z, wref.z, tol);
}

template<class VelocityField>
static void checkVorticity(const VelocityField& velocityField, int nsamples = 100, long seed = 424242L)
{
    constexpr real L = 10.0_r;
    std::mt19937 gen(seed);
    std::uniform_real_distribution<real> distr(-L, L);

    for (int i = 0; i < nsamples; ++i)
    {
        const real3 r {distr(gen), distr(gen), distr(gen)};
        checkVorticity(velocityField, r);
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


GTEST_TEST( VELOCITY_FIELD, vorticity_constant )
{
    const real3 v {-1.04_r, 2.3_r, 0.12_r};
    const VelocityFieldConstant vel{v};
    checkVorticity(vel);
}

GTEST_TEST( VELOCITY_FIELD, vorticity_none )
{
    const VelocityFieldNone vel{};
    checkVorticity(vel);
}

GTEST_TEST( VELOCITY_FIELD, vorticity_taylorGreenVortex )
{
    const real3 magn {1.0_r, 1.0_r, -2.0_r};
    const real3 invPeriod {0.3_r, 0.3_r, 0.3_r};
    const VelocityFieldTaylorGreenVortex vel{magn, invPeriod};
    checkVorticity(vel);
}

GTEST_TEST( VELOCITY_FIELD, sum_constants )
{
    const real3 v0 {-1.04_r, 2.3_r, 0.12_r};
    const real3 v1 {18.42_r, 3.2_r, 1.34_r};
    std::vector<std::unique_ptr<BaseVelocityField>> fields;
    fields.push_back(std::make_unique<VelocityFieldConstant>(v0));
    fields.push_back(std::make_unique<VelocityFieldConstant>(v1));

    const VelocityFieldSum vel(std::move(fields));

    checkDivergenceFree(vel);
    checkVorticity(vel);

    const auto v = vel.getVelocity({0.0_r, 0.0_r, 0.0_r}, 0.0_r);

    constexpr real tol = 1e-6_r;
    ASSERT_NEAR(v.x, v0.x + v1.x, tol);
    ASSERT_NEAR(v.y, v0.y + v1.y, tol);
    ASSERT_NEAR(v.z, v0.z + v1.z, tol);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
