#include "optimal_path.h"
#include "helpers.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include <korali.hpp>
#pragma GCC diagnostic pop

namespace msode {
namespace analytic_control {

std::vector<real3> computeA(const MatrixReal& U, const std::vector<real3>& positions)
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


real computeTime(const std::vector<real3>& A, real3 normal)
{
    real t {0._r};
    for (auto ai : A)
        t += std::fabs(dot(ai, normal));
    return t;
}

real computeTime(const std::vector<real3>& A, Quaternion q)
{
    const real3 e1 {1.0_r, 0.0_r, 0.0_r};
    const real3 e2 {0.0_r, 1.0_r, 0.0_r};
    const real3 e3 {0.0_r, 0.0_r, 1.0_r};

    return
        computeTime(A, q.rotate(e1)) +
        computeTime(A, q.rotate(e2)) +
        computeTime(A, q.rotate(e3));
}

static inline Quaternion paramsToQuaternion(const std::vector<double>& params)
{
    const real theta = static_cast<real>(params[0]);
    const real phi   = static_cast<real>(params[1]);
    const real psi   = static_cast<real>(params[2]);
    
    const real3 normal {std::cos(theta) * std::sin(phi),
                        std::sin(theta) * std::sin(phi),
                        std::cos(phi)};

    const auto q = Quaternion::createFromRotation(psi, normal);
    return q.normalized();
}

Quaternion findBestPath(const std::vector<real3>& A)
{
    std::function<void(korali::Sample&)> evaluatePath = [&](korali::Sample& k)
    {
        const auto q = paramsToQuaternion(k["Parameters"]);
        k["F(x)"] = -computeTime(A, q); // maximize -T
    };

    auto e = korali::Experiment();
    e["Problem"]["Type"] = "Optimization/Stochastic";
    e["Problem"]["Objective Function"] = evaluatePath;

    e["Solver"]["Type"] = "CMAES";
    e["Solver"]["Population Size"] = 32;
    e["Solver"]["Termination Criteria"]["Min Value Difference Threshold"] = 1e-7;
    e["Solver"]["Termination Criteria"]["Max Generations"] = 10000;

    e["Variables"][0]["Name"] = "theta";
    e["Variables"][0]["Lower Bound"] = -2 * M_PI;
    e["Variables"][0]["Upper Bound"] = +4 * M_PI;

    e["Variables"][1]["Name"] = "phi";
    e["Variables"][1]["Lower Bound"] =   - M_PI;
    e["Variables"][1]["Upper Bound"] = 2 * M_PI;

    e["Variables"][2]["Name"] = "psi";
    e["Variables"][2]["Lower Bound"] =   - M_PI;
    e["Variables"][2]["Upper Bound"] = 3 * M_PI;

    e["Console Output"]["Frequency"] = 0;
    e["Console Output"]["Verbosity"] = "Silent";
    e["Results"]["Frequency"] = 0;

    e["Random Seed"] = 424242;

    auto k = korali::Engine();
    k.run(e);

    return paramsToQuaternion(e["Solver"]["Best Ever Variables"]);
}

} // namespace analytic_control
} // namespace msode
