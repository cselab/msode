#include "simulation.h"
#include "factory.h"

#include <Eigen/LU>

#include <iostream>
#include <random>

using MatrixReal = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using ArrayReal  = Eigen::Array <real, Eigen::Dynamic, 1>;

constexpr real magneticFieldMagnitude {1.0_r};

static inline real computeStepOutFrequency(const RigidBody& body)
{
    const real m = length(body.magnMoment);
    const real Cxx = body.propulsion.C[0];
    return magneticFieldMagnitude * m * Cxx;
}

static inline auto computeStepOutFrequencies(const std::vector<RigidBody>& bodies)
{
    std::vector<real> omegas;
    omegas.reserve(bodies.size());
    for (const auto& b : bodies)
        omegas.push_back(computeStepOutFrequency(b));
    return omegas;
}

static inline real meanVelocity(real3 r0, real3 r1, real T)
{
    return length(r0-r1)/T;
}

static inline real computeMeanVelocity(RigidBody body, real omega)
{
    constexpr real tEnd = 200.0_r;
    constexpr real dt {1e-3_r};
    constexpr long nsteps = tEnd / dt;

    constexpr real3 rStart {0.0_r, 0.0_r, 0.0_r};
    body.r = rStart;
        
    auto omegaField        = [omega](real t) {return omega;};
    auto rotatingDirection = []     (real t) {return real3{1.0_r, 0.0_r, 0.0_r};};

    MagneticField magneticField {magneticFieldMagnitude, omegaField, rotatingDirection};
    const std::vector<RigidBody> rigidBodies {body};
    Simulation simulation {rigidBodies, magneticField};

    simulation.run(nsteps, dt);

    const real3 rEnd = simulation.getBodies()[0].r;

    return meanVelocity(rStart, rEnd, tEnd);
}


static inline real computeForwardVelocity(const RigidBody& body, real omega)
{
    const real omegaC = computeStepOutFrequency(body);

    if (omega <= omegaC)
    {
        const real Bxx = body.propulsion.B[0];
        const real Cxx = body.propulsion.C[0];
        return Bxx / Cxx * omega;
    }
    else
    {
        return computeMeanVelocity(body, omega);
    }
}

static MatrixReal createVelocityMatrix(const std::vector<RigidBody>& bodies)
{
    const size_t n = bodies.size();
    const auto omegas = computeStepOutFrequencies(bodies);
    
    MatrixReal V(n, n);

    for (size_t i = 0; i < n; ++i)
    {
        const auto& body = bodies[i];
        for (size_t j = 0; j < n; ++j)
        {
            const real omega = omegas[j];
            V(i, j) = computeForwardVelocity(body, omega);
        }
    }
    return V;
}

static std::vector<real3> generateRandomPositions(int n, real3 boxLo, real3 boxHi, long seed = 42)
{
    std::vector<real3> positions;
    positions.reserve(n);

    std::mt19937 gen(seed);
    std::uniform_real_distribution<real> distrx(boxLo.x, boxHi.x);
    std::uniform_real_distribution<real> distry(boxLo.y, boxHi.y);
    std::uniform_real_distribution<real> distrz(boxLo.z, boxHi.z);

    for (int i = 0; i < n; ++i)
    {
        real3 r;
        r.x = distrx(gen);
        r.y = distry(gen);
        r.z = distrz(gen);
        positions.push_back(r);
    }

    return positions;
}

static MatrixReal stackPositions(const std::vector<real3>& positions)
{
    const size_t n = positions.size();
    MatrixReal X(n, 3);

    for (size_t i = 0; i < n; ++i)
    {
        const real3 r = positions[i];
        X(i,0) = r.x;
        X(i,1) = r.y;
        X(i,2) = r.z;
    }

    return X;
}

static real3 findBestPlane(const MatrixReal& U, const MatrixReal& X)
{
    int n = U.rows();
    std::cout << n << "\n\n" << std::endl;
    MatrixReal A(U * X);

    MatrixReal normal(3,1), normalOld(3,1), signs(n,1);

    normal = A.colwise().sum().transpose();
    normal /= normal.norm();
    normalOld = normal;
    
    for (int step = 0; step < 10; ++step)
    {
        signs = A * normal;

        for (int i = 0; i < signs.rows(); ++i)
            signs(i,0) = signs(i,0) >= 0 ? 1.0_r : -1.0_r;
        
        normal = A.transpose() * signs;
        normal /= normal.norm();
        std::cout << (normal - normalOld).norm() << "\n";
        normalOld = normal;
    }

    return normalized({normal(0,0),
                       normal(1,0),
                       normal(2,0)});
}


int main(int argc, char **argv)
{
    if (argc < 2                     ||
        std::string(argv[1]) == "-h" ||
        std::string(argv[1]) == "--help")
    {
        fprintf(stderr, "usage : %s <swimmer0.cfg> <swimmer1.cfg>... \n\n", argv[0]);
        return 1;
    }

    const real3 boxLo{-50.0_r, -50.0_r, -50.0_r};
    const real3 boxHi{+50.0_r, +50.0_r, +50.0_r};
    
    std::vector<RigidBody> bodies;
    for (int i = 1; i < argc; ++i)
    {
        const RigidBody body = Factory::readRigidBodyConfig(argv[i]);
        bodies.push_back(body);
    }

    const MatrixReal V = createVelocityMatrix(bodies);
    const MatrixReal U = V.inverse();

    // std::cout << V << "\n\n";
    // std::cout << U << std::endl;

    for (int i = 0; i < 10; ++i)
    {
        std::cout << "=========== step " << i << std::endl;
        std::vector<real3> initialPositions = generateRandomPositions(bodies.size(), boxLo, boxHi, 42 * i + 13);
        auto stackedPositions = stackPositions(initialPositions);
        
        auto n = findBestPlane(U, stackedPositions);
        std::cout << n << std::endl;
    }
    return 0;
}
