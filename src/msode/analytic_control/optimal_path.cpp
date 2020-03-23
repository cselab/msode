#include "optimal_path.h"
#include "helpers.h"

#include <msode/utils/optimizers/cmaes.h>
#include <LBFGS.h>

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


real computeTravelTime(const std::vector<real3>& A, real3 direction)
{
    MSODE_Expect(std::fabs(length(direction) - 1.0_r) < 1e-6_r,
                 "Expect a direction with unit length, got %g %g %g",
                 direction.x, direction.y, direction.z);
    
    real t {0._r};
    for (auto ai : A)
        t += std::fabs(dot(ai, direction));
    return t;
}

real computeTravelTime(const std::vector<real3>& A, Quaternion q)
{
    constexpr real3 e1 {1.0_r, 0.0_r, 0.0_r};
    constexpr real3 e2 {0.0_r, 1.0_r, 0.0_r};
    constexpr real3 e3 {0.0_r, 0.0_r, 1.0_r};

    return
        computeTravelTime(A, q.rotate(e1)) +
        computeTravelTime(A, q.rotate(e2)) +
        computeTravelTime(A, q.rotate(e3));
}

/** \brief maps from euler angles to quaternion
 */
static inline Quaternion anglesToQuaternion(real theta, real phi, real psi)
{
    const real3 normal {std::cos(phi) * std::sin(psi),
                        std::sin(phi) * std::sin(psi),
                        std::cos(psi)};

    const auto q = Quaternion::createFromRotation(theta, normal);

    return q;
}

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

Quaternion findBestPathCMAES(const std::vector<real3>& A, bool verbose)
{
    using namespace utils;

    auto travelTime = [&](const CMAES::Vector& x) -> real
    {
        const auto q = anglesToQuaternion(x(0), x(1), x(2));
        return computeTravelTime(A, q);
    };

    const int lambda = 8;
    CMAES::Vector x = CMAES::Vector::Zero(3);
    const real sigma = 1.0_r;
    
    CMAES cma(travelTime, lambda, x, sigma, 424240);

    real val;
    std::tie(x, val) = cma.runMinimization(1e-3, 1000, verbose);

    return anglesToQuaternion(x(0), x(1), x(2));
}


real3 computeTravelTimeGradient(const std::vector<real3>& A, real theta, real phi, real psi)
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

class TravelTimeSmoothFunction
{
public:
    TravelTimeSmoothFunction(const std::vector<real3>& A, real epsilon) :
        A_(A),
        epsilon_(epsilon)
    {}

    real operator()(const Eigen::VectorXd& x, Eigen::VectorXd& grad) const
    {
        const real theta = x[0];
        const real phi = x[1];
        const real psi = x[2];
        
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

        real tt {0.0_r};
        grad[0] = grad[1] = grad[2] = 0;

        for (auto a : A_)
        {
            const real aR0 = dot(a, R0);
            const real aR1 = dot(a, R1);
            const real aR2 = dot(a, R2);

            // travel time
            tt += smoothAbs(aR0) + smoothAbs(aR1) + smoothAbs(aR2);

            // gradients
            // theta
            grad[0] += smoothSgn(aR0) * dot(a, Rth0) + smoothSgn(aR1) * dot(a, Rth1) + smoothSgn(aR2) * dot(a, Rth2);
            // phi
            grad[1] += smoothSgn(aR0) * dot(a, Rph0) + smoothSgn(aR1) * dot(a, Rph1) + smoothSgn(aR2) * dot(a, Rph2);
            // psi
            grad[2] += smoothSgn(aR0) * dot(a, Rps0) + smoothSgn(aR1) * dot(a, Rps1) + smoothSgn(aR2) * dot(a, Rps2);
        }

        return tt;
    }

private:
    real smoothAbs(real x) const
    {
        return std::sqrt(x*x + epsilon_);
    }

    real smoothSgn(real x) const
    {
        return x / smoothAbs(x);
    }
    
private:
    const std::vector<real3>& A_;
    const real epsilon_;
};

Quaternion findBestPathLBFGS(const std::vector<real3>& A)
{
    using Eigen::VectorXd;
    using namespace LBFGSpp;

    TravelTimeSmoothFunction func(A, 5e-1_r);
    
    constexpr int n = 3; // theta, phi, psi

    // Set up parameters
    LBFGSParam<real> param;
    param.epsilon = 1e-6_r;
    param.max_iterations = 5000;

    // Create solver and function object
    LBFGSSolver<real, LineSearchBracketing> solver(param);

    // Initial guess
    VectorXd x = VectorXd::Zero(n);
    // x will be overwritten to be the best point found
    real fx;
    // const int niter =
    solver.minimize(func, x, fx);

    // if (niter >= param.max_iterations)
    //     fprintf(stderr, "warning: did not converge in less that %d iterations\n", niter);

    return anglesToQuaternion(x[0], x[1], x[2]);
}




static inline Quaternion koraliParamsToQuaternion(const std::vector<double>& params)
{
    const real theta = static_cast<real>(params[0]);
    const real phi   = static_cast<real>(params[1]);
    const real psi   = static_cast<real>(params[2]);
    return anglesToQuaternion(theta, phi, psi);
}

Quaternion findBestPathCMAESKorali(const std::vector<real3>& A, bool verbose)
{
    auto evaluatePath = [&](korali::Sample& k)
    {
        const auto q = koraliParamsToQuaternion(k["Parameters"]);
        k["F(x)"] = -computeTravelTime(A, q); // maximize -T
    };

    auto e = korali::Experiment();
    e["Problem"]["Type"] = "Optimization/Stochastic";
    e["Problem"]["Objective Function"] = evaluatePath;

    e["Solver"]["Type"] = "CMAES";
    e["Solver"]["Population Size"] = 8;
    e["Solver"]["Termination Criteria"]["Min Value Difference Threshold"] = 1e-3;
    e["Solver"]["Termination Criteria"]["Max Generations"] = 1000;


    const real sigma = 1.0_r;
    
    e["Variables"][0]["Name"] = "theta";
    // e["Variables"][0]["Lower Bound"] =   - M_PI;
    // e["Variables"][0]["Upper Bound"] = 3 * M_PI;
    e["Variables"][0]["Initial Mean"] = 0.0;
    e["Variables"][0]["Initial Standard Deviation"] = sigma;


    e["Variables"][1]["Name"] = "phi";
    // e["Variables"][1]["Lower Bound"] = -2 * M_PI;
    // e["Variables"][1]["Upper Bound"] = +4 * M_PI;
    e["Variables"][1]["Initial Mean"] = 0.0;
    e["Variables"][1]["Initial Standard Deviation"] = sigma;


    e["Variables"][2]["Name"] = "psi";
    // e["Variables"][2]["Lower Bound"] =   - M_PI;
    // e["Variables"][2]["Upper Bound"] = 2 * M_PI;
    e["Variables"][2]["Initial Mean"] = 0.0;
    e["Variables"][2]["Initial Standard Deviation"] = sigma;

    if (!verbose)
    {
        e["Console Output"]["Frequency"] = 0;
        e["Console Output"]["Verbosity"] = "Silent";
        e["Results"]["Frequency"] = 0;
    }

    e["Random Seed"] = 424242;

    auto k = korali::Engine();
    k.run(e);

    return koraliParamsToQuaternion(e["Solver"]["Best Ever Variables"]);
}
 


} // namespace analytic_control
} // namespace msode
