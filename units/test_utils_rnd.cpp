#include <msode/utils/rnd.h>

#include <gtest/gtest.h>
#include <cmath>

using namespace msode;

// https://www.codeproject.com/Articles/432194/How-to-Calculate-the-Chi-Squared-P-Value
static inline real incompleteGamma(real s, real z)
{
    if (z < 0.0_r)
	return 0.0_r;

    const real sc = std::pow(z, s) * std::exp(-z) / s;

    real sum = 1.0_r;
    real nom = 1.0_r;
    real denom = 1.0_r;

    constexpr int niters = 200;
    
    for (int i = 0; i < niters; ++i)
    {
	nom *= z;
	s += 1.0_r;
	denom *= s;
	sum += (nom / denom);
    }
 
    return sum * sc;
}

static inline real chisqr(int Dof, real Cv)
{
    if (Cv < 0 || Dof < 1)
        return 0.0_r;

    const real k = ((real) Dof) * 0.5_r;
    const real x = Cv * 0.5_r;

    if (Dof == 2)
	return std::exp(-x);
 
    real PValue = incompleteGamma(k, x);

    if (std::isnan(PValue) || std::isinf(PValue))
    {
        return 1e-14_r;
    } 

    PValue /= std::tgamma(k);
	
    return (1.0_r - PValue);
}

GTEST_TEST( chiSquareTest, uniform)
{
    const real a = 0.0_r;
    const real b = 2.0_r;

    constexpr int nbins = 10;
    const real h = (b - a) / nbins;
    std::vector<int> counts(nbins, 0);

    const long nsamples = 10000;
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
        const int oi = counts[i];
        const int ei = nsamples  / real(nbins);

        const auto di = oi - ei;
        D += static_cast<real>(di*di) / static_cast<real>(ei);
    }
    D /= nbins;
    
    const real alpha = 0.05;

    // FILE *f = fopen("tmp.dat", "w");
    // for (int i = 0; i < 100; ++i)
    // {
    //     real x = i * 0.5;
    //     real y = chisqr(100, x);
    //     fprintf(f, "%g %g\n", x, y);
    // }
    // fclose(f);

    //printf("%g %g\n", D, chisqr(nbins-1, D));
    ASSERT_GE(chisqr(nbins-1, D), 1.0_r-alpha);
}

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

GTEST_TEST( rnd, shell )
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

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
