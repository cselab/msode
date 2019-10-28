#include "ac/helpers.h"

#include "simulation.h"
#include "factory.h"

#include <Eigen/LU>
#include <korali.hpp>

#include <iostream>
#include <random>

using MatrixReal = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
using ArrayReal  = Eigen::Array <real, Eigen::Dynamic, 1>;

static inline real computeStepOutFrequency(real magneticFieldMagnitude, const RigidBody& body)
{
    const real m = length(body.magnMoment);
    const real Cxx = body.propulsion.C[0];
    return magneticFieldMagnitude * m * Cxx;
}

static inline auto computeStepOutFrequencies(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies)
{
    std::vector<real> omegas;
    omegas.reserve(bodies.size());
    for (const auto& b : bodies)
        omegas.push_back(computeStepOutFrequency(magneticFieldMagnitude, b));
    return omegas;
}

static inline real meanVelocity(real3 r0, real3 r1, real T)
{
    return length(r0-r1)/T;
}

static inline real computeMeanVelocity(RigidBody body, real magneticFieldMagnitude, real omega)
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


static inline real computeForwardVelocity(const RigidBody& body, real magneticFieldMagnitude, real omega)
{
    const real omegaC = computeStepOutFrequency(magneticFieldMagnitude, body);

    if (omega <= omegaC)
    {
        const real Bxx = body.propulsion.B[0];
        const real Cxx = body.propulsion.C[0];
        return Bxx / Cxx * omega;
    }
    else
    {
        return computeMeanVelocity(body, magneticFieldMagnitude, omega);
    }
}

static MatrixReal createVelocityMatrix(real magneticFieldMagnitude, const std::vector<RigidBody>& bodies)
{
    const size_t n = bodies.size();
    const auto omegas = computeStepOutFrequencies(magneticFieldMagnitude, bodies);
    
    MatrixReal V(n, n);

    for (size_t i = 0; i < n; ++i)
    {
        const auto& body = bodies[i];
        for (size_t j = 0; j < n; ++j)
        {
            const real omega = omegas[j];
            V(i, j) = computeForwardVelocity(body, magneticFieldMagnitude, omega);
        }
    }
    return V;
}

static std::vector<real3> computeA(const MatrixReal& U, const std::vector<real3>& positions)
{
    const size_t n = positions.size();
    std::vector<real3> A;
    A.reserve(n);

    for (size_t i = 0; i < n; ++i)
    {
        auto ai = make_real3(0.0_r);
        
        for (size_t j = 0; j < n; ++j)
            ai += U(i,j) * positions[j];

        A.push_back(ai);
    }

    return A;
}

// because korali passes functions through json... (WTF!?)
// need to use global variables

std::vector<real3> AGlobal;

static inline real computeTime(real3 normal)
{
    real t {0._r};
    for (auto ai : AGlobal)
        t += std::fabs(dot(ai, normal));
    return t;
}

static inline real3 paramsToNormal(const std::vector<double>& params)
{
    const real theta = static_cast<real>(params[0]);
    const real phi   = static_cast<real>(params[1]);
    
    const real3 normal {std::cos(theta) * std::sin(phi),
                        std::sin(theta) * std::sin(phi),
                        std::cos(phi)};
    return normal;
}
    
void evaluate(korali::Sample& k)
{
    const real3 normal = paramsToNormal(k["Parameters"]);
    k["Evaluation"] = computeTime(normal);
}

static real3 findBestPlane(const std::vector<real3>& A)
{
    AGlobal = A;

    auto k = korali::Engine();

    k["Problem"]["Type"] = "Evaluation/Direct/Basic";
    k["Problem"]["Objective"] = "Minimize";
    k["Problem"]["Objective Function"] = &evaluate;

    k["Solver"]["Type"] = "Optimizer/CMAES";
    k["Solver"]["Population Size"] = 32;
    k["Solver"]["Termination Criteria"]["Min Value Difference Threshold"] = 1e-7;
    k["Solver"]["Termination Criteria"]["Max Generations"] = 100;

    k["Variables"][0]["Name"] = "theta";
    k["Variables"][0]["Lower Bound"] = -2 * M_PI;
    k["Variables"][0]["Upper Bound"] = +4 * M_PI;

    k["Variables"][1]["Name"] = "phi";
    k["Variables"][1]["Lower Bound"] =   - M_PI;
    k["Variables"][1]["Upper Bound"] = 2 * M_PI;

    k["Console Output"]["Frequency"] = 10;
    k["Results Output"]["Frequency"] = 10;
    
    k.runSingle();

    return paramsToNormal(k["Solver"]["Internal"]["Best Ever Variables"]);
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

    // std::cout << V << "\n\n";
    // std::cout << U << std::endl;

    for (int i = 0; i < 1; ++i)
    {
        std::cout << "=========== step " << i << std::endl;
        std::vector<real3> initialPositions = generateRandomPositions(bodies.size(), boxLo, boxHi, 42 * i + 13);
        auto A = computeA(U, initialPositions);
        
        auto n = findBestPlane(A);
        std::cout << n << "\t\t" << computeTime(n) << std::endl;
    }
    return 0;
}
