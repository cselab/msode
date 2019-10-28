#include "optimal_path.h"
#include "helpers.h"

#include <korali.hpp>

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

// because korali passes functions through json... (WTF!?)
// need to use global variables

std::vector<real3> AGlobal;

real computeTime(real3 normal)
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
    
static void evaluate(korali::Sample& k)
{
    const real3 normal = paramsToNormal(k["Parameters"]);
    k["Evaluation"] = computeTime(normal);
}

real3 findBestPlane(const std::vector<real3>& A)
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





real computeTime(Quaternion q)
{
    const real3 e1 {1.0_r, 0.0_r, 0.0_r};
    const real3 e2 {0.0_r, 1.0_r, 0.0_r};
    const real3 e3 {0.0_r, 0.0_r, 1.0_r};

    return
        computeTime(q.rotate(e1)) +
        computeTime(q.rotate(e2)) +
        computeTime(q.rotate(e3));
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
    
static void evaluatePath(korali::Sample& k)
{
    const auto q = paramsToQuaternion(k["Parameters"]);
    k["Evaluation"] = computeTime(q);
}

Quaternion findBestPath(const std::vector<real3>& A)
{
    AGlobal = A;

    auto k = korali::Engine();

    k["Problem"]["Type"] = "Evaluation/Direct/Basic";
    k["Problem"]["Objective"] = "Minimize";
    k["Problem"]["Objective Function"] = &evaluatePath;

    k["Solver"]["Type"] = "Optimizer/CMAES";
    k["Solver"]["Population Size"] = 32;
    k["Solver"]["Termination Criteria"]["Min Value Difference Threshold"] = 1e-7;
    k["Solver"]["Termination Criteria"]["Max Generations"] = 10000;

    k["Variables"][0]["Name"] = "theta";
    k["Variables"][0]["Lower Bound"] = -2 * M_PI;
    k["Variables"][0]["Upper Bound"] = +4 * M_PI;

    k["Variables"][1]["Name"] = "phi";
    k["Variables"][1]["Lower Bound"] =   - M_PI;
    k["Variables"][1]["Upper Bound"] = 2 * M_PI;

    k["Variables"][2]["Name"] = "psi";
    k["Variables"][2]["Lower Bound"] =   - M_PI;
    k["Variables"][2]["Upper Bound"] = 3 * M_PI;

    k["Console Output"]["Frequency"] = 10;
    k["Results Output"]["Frequency"] = 10;
    
    k.runSingle();

    return paramsToQuaternion(k["Solver"]["Internal"]["Best Ever Variables"]);
}
