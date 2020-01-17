#include <msode/utils/rnd.h>

#include <gtest/gtest.h>
#include <cmath>

using namespace msode;

constexpr real chiSq_99_5 = 123.23;

GTEST_TEST( chiSquareTest, uniform)
{
    const real a = 0.0_r;
    const real b = 2.0_r;

    constexpr int nbins = 100;
    const real h = (b - a) / nbins;
    std::vector<int> counts(nbins, 0);

    const long nsamples = 100000;
    std::mt19937 gen(4242);
    std::uniform_real_distribution<real> uniform(a, b);
    
    for (long i = 0; i < nsamples; ++i)
    {
        const auto s = uniform(gen);
        const int id = (s - a) / h;
        ++ counts[id];
    }

    real D = 0.0_r;
    for (int i = 0; i < nbins; ++i)
    {
        const real oi = counts[i];
        const real ei = nsamples  / real(nbins);

        const auto di = oi - ei;
        D += di*di / ei;
    }
    
    ASSERT_LE( D, chiSq_99_5 );
}

GTEST_TEST( rnd, samples_are_in_box )
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

GTEST_TEST( rnd, samples_are_in_ball )
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

GTEST_TEST( rnd, ball_samples_are_uniform )
{
    std::mt19937 gen(4242);
    const real R {5.0_r};

    constexpr long nSamples = 100000;
    constexpr int nbins = 100;

    const real h = R / nbins;
    std::vector<int> radialHist(nbins, 0);

    for (int i = 0; i < nSamples; ++i)
    {
        const real3 x = utils::generateUniformPositionBall(gen, R);
        const real r = length(x);

        const int id = r / h;
        ++radialHist[id];
    }

    real D {0.0_r};
    const real totVolume = 4.0_r * M_PI / 3.0_r * R*R*R;
    
    for (int i = 0; i < nbins; ++i)
    {
        const real r0 = i * h;
        const real r1 = r0 + h;

        const real volumei = 4.0_r * M_PI / 3.0_r * (r1*r1*r1 - r0*r0*r0);

        const real ei = nSamples * volumei / totVolume;
        const real oi = radialHist[i];
        const real di = ei - oi;
        D += di * di / ei;
    }

    ASSERT_LE( D, chiSq_99_5 );
}

GTEST_TEST( rnd, samples_are_in_shell )
{
    std::mt19937 gen(4242);
    const real R1 {2.0_r};
    const real R2 {6.0_r};

    const long nSamples = 1000;

    for (int i = 0; i < nSamples; ++i)
    {
        const auto r = utils::generateUniformPositionShell(gen, R1, R2);
        ASSERT_GE(dot(r,r), R1*R1);
        ASSERT_LE(dot(r,r), R2*R2);
    }
}

GTEST_TEST( rnd, shell_samples_are_uniform )
{
    std::mt19937 gen(4242);
    const real R1 {2.0_r};
    const real R2 {6.0_r};

    constexpr long nSamples = 100000;
    constexpr int nbins = 100;

    const real h = (R2 - R1) / nbins;
    std::vector<int> radialHist(nbins, 0);

    for (int i = 0; i < nSamples; ++i)
    {
        const real3 x = utils::generateUniformPositionShell(gen, R1, R2);
        const real r = length(x);

        const int id = (r - R1) / h;
        ++radialHist[id];
    }

    real D {0.0_r};
    const real totVolume = 4.0_r * M_PI / 3.0_r * (R2*R2*R2 - R1*R1*R1);
    
    for (int i = 0; i < nbins; ++i)
    {
        const real r0 = R1 + i * h;
        const real r1 = r0 + h;

        const real volumei = 4.0_r * M_PI / 3.0_r * (r1*r1*r1 - r0*r0*r0);

        const real ei = nSamples * volumei / totVolume;
        const real oi = radialHist[i];
        const real di = ei - oi;
        D += di * di / ei;
    }

    ASSERT_LE( D, chiSq_99_5 );
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
