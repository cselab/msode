#pragma once

#include <msode/core/math.h>
#include <msode/core/types.h>

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <functional>
#include <random>
#include <vector>

namespace msode {
namespace utils {

/**
   See https://arxiv.org/pdf/1604.00772.pdf p.36
 */
class CMAES
{
public:
    using Vector = Eigen::Matrix<real, Eigen::Dynamic, 1>;
    using Matrix = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;

    using Function = std::function<real(Vector)>;

    CMAES(const Function& function, int lambda, Vector mean, real sigma, long seed);

    void runGeneration();

private:
    Vector _generateNormalDistrVector();
    void _computeOrdering(const std::vector<real>& values);
    
private:
    Function function_;

    std::mt19937 gen_;
    std::normal_distribution<real> normDistr_{0.0_r, 1.0_r};
    
    int n_;
    int countEval_ {0};
    
    int lambda_;
    int mu_;
    real muEff_;
    real sigma_;
    Vector xmean_;
    Matrix C_;
    Vector pC_;
    Vector pSigma_;

    real cC_;
    real cSigma_;
    real c1_;
    real cMu_;
    real dSigma_;
    real chiSquareNumber_; ///< expectation of || N(0,I) ||
    std::vector<real> weights_;

    Eigen::EigenSolver<Matrix> CDecomposition_;

    std::vector<int> order_;
    std::vector<Vector> samples_;
    std::vector<Vector> ys_;
    std::vector<Vector> zs_;
    std::vector<real> functionValues_;
};

} // namespace utils
} // namespace msode

