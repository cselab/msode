#include <msode/core/quaternion.h>

#include <gtest/gtest.h>
#include <random>

using namespace msode;

constexpr real3 ex {1.0_r, 0.0_r, 0.0_r};
constexpr real3 ey {0.0_r, 1.0_r, 0.0_r};
constexpr real3 ez {0.0_r, 0.0_r, 1.0_r};

static inline void requireEquals(real3 a, real3 b, real eps = 1e-6_r)
{
    ASSERT_NEAR(a.x, b.x, eps);
    ASSERT_NEAR(a.y, b.y, eps);
    ASSERT_NEAR(a.z, b.z, eps);
}

GTEST_TEST( ROTATION, identity )
{
    auto q = Quaternion::createFromComponents(1, 0, 0, 0);
    auto v = q.rotate(ex);
    requireEquals(v, ex); 
}

GTEST_TEST( ROTATION, around_axis )
{
    {
        const auto q = Quaternion::createFromRotation(M_PI, ey);
        const auto v = q.rotate(ex);
        
        requireEquals(v, {-1.0_r, 0.0_r, 0.0_r}); 
    }
    {
        const auto q = Quaternion::createFromRotation(-0.5_r * M_PI, ey);
        const auto v = q.rotate(ex);
        
        requireEquals(v, ez);
    }
}

static inline real3 makeRandomUnitVector(std::mt19937& gen)
{
     std::uniform_real_distribution<real> U(0.0_r, 1.0_r);
     const real theta = 2.0_r * M_PI * U(gen);
     const real phi   = std::acos(1.0_r - 2.0_r * U(gen));

     return {std::sin(phi) * std::cos(theta),
             std::sin(phi) * std::sin(theta),
             std::cos(phi)};
}

GTEST_TEST( CONSTRUCTION, from_random_vectors )
{
    const unsigned long seed = 424242;
    const int numTries = 50;
    std::mt19937 gen(seed);

    for (int i = 0; i < numTries; ++i)
    {
        const auto u = makeRandomUnitVector(gen);
        const auto v = makeRandomUnitVector(gen);

        const auto q = Quaternion::createFromVectors(u,v);
        requireEquals(v, q.rotate(u));
    }
}

GTEST_TEST( CONSTRUCTION, from_aligned_vectors )
{
    {
        const real3 u = ex;
        const real3 v = ex;

        const auto q = Quaternion::createFromVectors(u,v);
        requireEquals(v, q.rotate(u));
    }
    {
        const real3 u = ex;
        const real3 v = {-1.0_r, 0.0_r, 0.0_r};

        const auto q = Quaternion::createFromVectors(u,v);
        requireEquals(v, q.rotate(u));
    }
}

static inline real dot(const std::array<real,3>& a, real3 b)
{
    return a[0] * b.x + a[1] * b.y + a[2] * b.z;
}

static inline real3 multiply(const RotMatrix& R, real3 v)
{
    return {dot(R[0], v),
            dot(R[1], v),
            dot(R[2], v)};
}

static inline void checkSameRotation(const RotMatrix& R, const Quaternion& q)
{
    requireEquals(q.rotate(ex), multiply(R, ex));
    requireEquals(q.rotate(ey), multiply(R, ey));
    requireEquals(q.rotate(ez), multiply(R, ez));
}

static inline Quaternion generateRandomQuaternion(std::mt19937& gen)
{
    const auto u = makeRandomUnitVector(gen);
    const auto v = makeRandomUnitVector(gen);

    return Quaternion::createFromVectors(u,v);
}

GTEST_TEST( CONSTRUCTION, from_random_rotation_matrix )
{
    const unsigned long seed = 424242;
    const int numTries = 50;
    std::mt19937 gen(seed);

    for (int i = 0; i < numTries; ++i)
    {
        const auto q = generateRandomQuaternion(gen);
        const auto R = q.getRotationMatrix();
        checkSameRotation(R, q);
    }
}

static inline RotMatrix generateRandomRotMatrix(std::mt19937& gen)
{
    const auto q = generateRandomQuaternion(gen);
    return q.getRotationMatrix();
}


GTEST_TEST( CONSTRUCTION, from_rotation_matrix )
{
    // identity
    {
        const std::array<real, 3> row0 {1.0_r, 0.0_r, 0.0_r};
        const std::array<real, 3> row1 {0.0_r, 1.0_r, 0.0_r};
        const std::array<real, 3> row2 {0.0_r, 0.0_r, 1.0_r};
        const RotMatrix identityMatrix {row0, row1, row2};
        
        const auto q = Quaternion::createFromMatrix(identityMatrix);
        requireEquals(ex, q.rotate(ex));
        requireEquals(ey, q.rotate(ey));
        requireEquals(ez, q.rotate(ez));
    }

    // random matrices
    {    
        const unsigned long seed = 424242;
        const int numTries = 50;
        std::mt19937 gen(seed);

        for (int i = 0; i < numTries; ++i)
        {
            const auto R = generateRandomRotMatrix(gen);
            const auto q = Quaternion::createFromMatrix(R);
        
            checkSameRotation(R, q);
        }
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
