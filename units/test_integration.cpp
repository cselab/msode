#include <msode/utils/integrator.h>

#define CATCH_CONFIG_MAIN
#include <catch2/catch.hpp>

using msode::real;
using namespace msode::literals;

TEST_CASE( "Linear polynomial", "[numerical integral]" )
{
    auto f = [](real x) {return 2.0_r * x + 5.0_r;};
    const real Iexact = 28.0_r;
    const real Itrap  = integrateTrapez(f, -1.0_r, 3.0_r, 1000);
    REQUIRE(Itrap == Approx(Iexact));
}

TEST_CASE( "Quadratic polynomial", "[numerical integral]" )
{
    auto f = [](real x) {return x*x - 3.0_r * x + 1.0_r;};
    const real Iexact = 10.5_r;
    const real Itrap  = integrateTrapez(f, -2.0_r, 1.0_r, 1000);
    REQUIRE(Itrap == Approx(Iexact));
}

