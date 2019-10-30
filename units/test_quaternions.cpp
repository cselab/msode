#include "quaternion.h"

#define CATCH_CONFIG_MAIN
#include "catch.hpp"

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

static inline void requireEquals(real3 a, real3 b, real eps = 1e-6_r)
{
    REQUIRE(a.x == Approx(b.x).margin(eps));
    REQUIRE(a.y == Approx(b.y).margin(eps));
    REQUIRE(b.z == Approx(b.z).margin(eps));
}

TEST_CASE( "Identity rotation", "[rotation]" )
{
    auto q = Quaternion::createFromComponents(1, 0, 0, 0);
    auto v = q.rotate(ex);
    requireEquals(v, ex); 
}

TEST_CASE( "Rotations around axis", "[rotation]" )
{
    {
        const auto q = Quaternion::createFromRotation(M_PI, ey);
        const auto v = q.rotate(ex);
        
        requireEquals(v, {-1.0_r, 0.0_r, 0.0_r}); 
    }
    {
        const auto q = Quaternion::createFromRotation(M_PI/2.0_r, ey);
        const auto v = q.rotate(ex);
        
        requireEquals(v, ez);
    }
}

