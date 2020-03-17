#include "helpers.h"

#include <msode/analytic_control/helpers.h>
#include <msode/analytic_control/optimal_path.h>

#include <cstdio>
#include <gtest/gtest.h>
#include <vector>

using namespace msode;

constexpr real magneticFieldMagnitude {1.0_r};

static std::vector<RigidBody> generateRandomBodies(int n, std::mt19937& gen)
{
    std::vector<RigidBody> bodies;
    bodies.reserve(n);

    for (int i = 0; i < n; ++i)
        bodies.push_back(helpers::generateRandomBody(gen));

    return bodies;
}

static std::vector<real3> generateA(int n, std::mt19937& gen)
{
    const auto bodies = generateRandomBodies(n, gen);
    const auto V = analytic_control::createVelocityMatrix(magneticFieldMagnitude, bodies);
    const auto U = V.inverse();

    const real3 boxLo {-50.0_r, -50.0_r, -50.0_r};
    const real3 boxHi {+50.0_r, +50.0_r, +50.0_r};
    const auto positions = analytic_control::generateRandomPositionsBox(n, boxLo, boxHi);

    return analytic_control::computeA(U, positions);
}

static inline Quaternion quaternionFromAngles(real theta, real phi, real psi)
{
    const real3 normal {std::cos(phi) * std::sin(psi),
                        std::sin(phi) * std::sin(psi),
                        std::cos(psi)};

    const auto q = Quaternion::createFromRotation(theta, normal);
    return q.normalized();
}

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

static real3 computeGradient(const std::vector<real3>& A, real theta, real phi, real psi)
{
    const real ct = std::cos(theta);
    const real st = std::sin(theta);

    const real omct = 1.0_r - ct;
    
    const real cph = std::cos(phi);
    const real sph = std::sin(phi);

    const real cps = std::cos(psi);
    const real sps = std::sin(psi);
    
    const real3 u {cph * sps, sph * sps, cps};

    const real3 uph {-sph * sps, cph * sps, 0.0_r};
    const real3 ups {cph * cps, sph * cps, -sps};
    
    // The rotation matrix columns

    const real3 R0 {u.x * u.x * omct + ct,         u.y * u.x * omct + u.z * st,   u.z * u.x * omct - u.y * st};
    const real3 R1 {u.x * u.y * omct - u.z * st,   u.y * u.y * omct + ct,         u.z * u.y * omct + u.x * st};
    const real3 R2 {u.x * u.z * omct + u.y * st,   u.y * u.z * omct - u.x * st,   u.z * u.z * omct + ct      };
    
    // derivatives of the columns of the rotation matrix w.r.t. theta, phi and psi
    
    const real3 Rth0 {u.x * u.x * st - st,         u.y * u.x * st + u.z * ct,   u.z * u.x * st - u.y * ct};
    const real3 Rth1 {u.x * u.y * st - u.z * ct,   u.y * u.y * st - st,         u.z * u.y * st + u.x * ct};
    const real3 Rth2 {u.x * u.z * st + u.y * ct,   u.y * u.z * st - u.x * ct,   u.z * u.z * st - st      };

    const real3 Rph0 {2 * uph.x * u.x * omct,         (uph.y * u.x + u.y * uph.x) * omct + uph.z * st,   (uph.z * u.x + u.z * uph.x) * omct - uph.y * st};
    const real3 Rph1 {(uph.x * u.y + u.x * uph.y) * omct - uph.z * st,   2 * uph.y * u.y * omct,         (uph.z * u.y + u.z * uph.y) * omct + uph.x * st};
    const real3 Rph2 {(uph.x * u.z + u.x * uph.z) * omct + uph.y * st,   (uph.y * u.z + u.y * uph.z) * omct - uph.x * st,   2 * uph.z * u.z * omct      };

    const real3 Rps0 {2 * ups.x * u.x * omct,         (ups.y * u.x + u.y * ups.x) * omct + ups.z * st,   (ups.z * u.x + u.z * ups.x) * omct - ups.y * st};
    const real3 Rps1 {(ups.x * u.y + u.x * ups.y) * omct - ups.z * st,   2 * ups.y * u.y * omct,         (ups.z * u.y + u.z * ups.y) * omct + ups.x * st};
    const real3 Rps2 {(ups.x * u.z + u.x * ups.z) * omct + ups.y * st,   (ups.y * u.z + u.y * ups.z) * omct - ups.x * st,   2 * ups.z * u.z * omct      };

    real3 gradient {0.0_r, 0.0_r, 0.0_r};

    for (auto a : A)
    {
        const real aR0 = dot(a, R0);
        const real aR1 = dot(a, R1);
        const real aR2 = dot(a, R2);

        // theta
        gradient.x += sgn(aR0) * dot(a, Rth0) + sgn(aR1) * dot(a, Rth1) + sgn(aR2) * dot(a, Rth2);
        // phi
        gradient.y += sgn(aR0) * dot(a, Rph0) + sgn(aR1) * dot(a, Rph1) + sgn(aR2) * dot(a, Rph2);
        // psi
        gradient.z += sgn(aR0) * dot(a, Rps0) + sgn(aR1) * dot(a, Rps1) + sgn(aR2) * dot(a, Rps2);
    }

    return gradient;
}

GTEST_TEST( AC_OPT, derivative )
{
    const int n = 4;
    const int numTries = 5;
    std::mt19937 gen {4217};
    const auto A = generateA(n, gen);

    auto F = [&](real theta, real phi, real psi)
    {
        return analytic_control::computeTime(A, quaternionFromAngles(theta, phi, psi));
    };

    std::uniform_real_distribution<real> dstr(0.0_r, 1.0_r);
    
    for (int i = 0; i < numTries; ++i)
    {
        const real theta = dstr(gen) * 2 * M_PI;
        const real phi   = dstr(gen) * 2 * M_PI;
        const real psi   = dstr(gen) * 1 * M_PI;

        const real h = 1e-4_r;
        const real tolerance = 1e-3_r;

        const real3 gradient_FD = {(F(theta + h, phi, psi) - F(theta - h, phi, psi)) / (2*h),
                                   (F(theta, phi + h, psi) - F(theta, phi - h, psi)) / (2*h),
                                   (F(theta, phi, psi + h) - F(theta, phi, psi - h)) / (2*h)};

        const real3 gradient = computeGradient(A, theta, phi, psi);

        ASSERT_NEAR(gradient_FD.x, gradient.x, tolerance);
        ASSERT_NEAR(gradient_FD.y, gradient.y, tolerance);
        ASSERT_NEAR(gradient_FD.z, gradient.z, tolerance);

        // printf("%g  %g\n", gradient_FD.x, gradient.x);
        // printf("%g  %g\n", gradient_FD.y, gradient.y);
        // printf("%g  %g\n\n", gradient_FD.z, gradient.z);
    }
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
