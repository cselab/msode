#include "ac/helpers.h"
#include "ac/optimal_path.h"

#include <algorithm>
#include <iostream>

static inline std::vector<real> computeBetas(const std::vector<real3>& initialPositions,
                                             const MatrixReal& U, real3 dir)
{
    const size_t n = initialPositions.size();
    std::vector<real> betas(n, 0.0_r);
    
    for (size_t i = 0; i < n; ++i)
        for (size_t j = 0; j < n; ++j)
        {
            const real xn = dot(initialPositions[j], dir);
                betas[i] += U(i,j) * xn;
        }
    return betas;
}

// static inline void print(const std::vector<real>& vec)
// {
//     for (auto v : vec)
//         std::cout << v << "\n";
//     std::cout << std::endl;
// }

static void simulateOptimalPath(real magneticFieldMagnitude,
                                std::vector<RigidBody> bodies, // by copy because will be modified (IC)
                                const std::vector<real3>& initialPositions,
                                const MatrixReal& U)
{
    const auto A = computeA(U, initialPositions);
    const Quaternion q = findBestPath(A);

    const std::vector<real> omegas = computeStepOutFrequencies(magneticFieldMagnitude, bodies);

    for (size_t i = 0; i < bodies.size(); ++i)
    {
        bodies[i].r = initialPositions[i];
        bodies[i].q = q;
    }
    
    
    const real3 e1 {1.0_r, 0.0_r, 0.0_r};
    const real3 e2 {0.0_r, 1.0_r, 0.0_r};
    const real3 e3 {0.0_r, 0.0_r, 1.0_r};

    const real3 dir1 = q.rotate(e1);
    const real3 dir2 = q.rotate(e2);
    const real3 dir3 = q.rotate(e3);

    const auto betas1 = computeBetas(initialPositions, U, dir1);
    const auto betas2 = computeBetas(initialPositions, U, dir2);
    const auto betas3 = computeBetas(initialPositions, U, dir3);

    // print(betas1);
    // print(betas2);
    // print(betas3);
    
    const real t1 = computeTime(A, dir1);
    const real t2 = computeTime(A, dir2);
    const real t3 = computeTime(A, dir3);

    const real scan1 = t1;
    const real scan2 = scan1 + t2;
    // const real scan3 = scan2 + t3;

    auto getOmega = [&](real t, const std::vector<real>& betas)
    {
        real tcum = 0.0_r;
        for (size_t i = 0; i < betas.size(); ++i)
        {
            const real beta = betas[i];
            tcum += fabs(beta);
            if (tcum > t)
            {
                return (beta > 0)
                    ? +omegas[i]
                    : -omegas[i];
            }
        }
        return 0.0_r; // default, should not happen
    };

    std::function<real(real)> omega = [&](real t)
    {
        if      (t < scan1) return getOmega(t,       betas1);
        else if (t < scan2) return getOmega(t-scan1, betas2);
        else                return getOmega(t-scan2, betas3);
    };

    std::function<real3(real)> rotatingDirection = [&](real t)
    {
        if      (t < scan1) return dir1;
        else if (t < scan2) return dir2;
        else                return dir3;
    };
    
    MagneticField field(magneticFieldMagnitude, omega, rotatingDirection);
    Simulation sim(bodies, field);

    const real tTot = t1 + t2 + t3;
    const real omegaMax = *std::max_element(omegas.begin(), omegas.end());
    
    const real dt = 1.0_r / (omegaMax * 20);
    const long nsteps = static_cast<long>(tTot/dt);

    std::cout << "nsteps = " << nsteps << std::endl;

    sim.activateDump("optimal_trajectories.txt", nsteps / 1000);

    sim.run(nsteps, dt);
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

    const real magneticFieldMagnitude = 1.0_r;

    const real3 boxLo{-50.0_r, -50.0_r, -50.0_r};
    const real3 boxHi{+50.0_r, +50.0_r, +50.0_r};
    
    std::vector<RigidBody> bodies;
    for (int i = 1; i < argc; ++i)
    {
        const RigidBody body = Factory::readRigidBodyConfig(argv[i]);
        bodies.push_back(body);
    }

    const MatrixReal V = createVelocityMatrix(magneticFieldMagnitude, bodies);
    const MatrixReal U = V.inverse();

    std::cout << V << "\n\n";
    std::cout << U << std::endl;

    const long seed = 42424242;
    std::vector<real3> initialPositions = generateRandomPositions(bodies.size(), boxLo, boxHi, seed);
    simulateOptimalPath(magneticFieldMagnitude, bodies, initialPositions, U);
    return 0;
}
