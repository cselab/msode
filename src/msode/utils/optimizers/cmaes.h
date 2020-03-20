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

/** Simple (mu lambda) CMA-ES for minimization
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
    /// generate a vector of size n_ whose entries are normally distrbuted N(0,1)
    Vector _generateNormalDistrVector();
    
    /// set the order_ array to contain the indices of \p values ordered from lowest to highest
    void _computeOrdering(const std::vector<real>& values);
    
private:
    Function function_; ///< objective function to minimize

    std::mt19937 gen_; ///< helper to generate random numbers
    std::normal_distribution<real> normDistr_{0.0_r, 1.0_r}; ///< normal distribution
    
    int n_; ///< problem dimension
    int countEval_ {0}; ///< total number of function evaluations
    
    int lambda_;    ///< population size
    int mu_;        ///< selection size
    real muEff_;    ///< variance-effective size of mu
    real sigma_;    ///< step size
    Vector xmean_;  ///< current distribution mean
    Matrix C_;      ///< current covariance matrix

    Vector pC_;     ///< evolution paths for C
    Vector pSigma_; ///< evolution paths for sigma

    real cC_;      ///< time constant for cumulation for C
    real cSigma_;  ///< time constant for cumulation for sigma
    real c1_;      ///< learning rate for rank-one update of C
    real cMu_;     ///< learning rate for rank-mu update of C
    real dSigma_;  ///< damping for sigma
    real chiSquareNumber_; ///< expectation of || N(0,I) ||

    std::vector<real> weights_; ///< normalize recombination weights array

    Eigen::EigenSolver<Matrix> CDecomposition_; ///< helper class to compute eigen decomposition of C

    std::vector<int> order_;       ///< ordering of the best candidates (best has index order_[0])
    std::vector<Vector> samples_;  ///< list of samples for new candidates (in evaluation space)
    std::vector<Vector> ys_;       ///< (list of samples minus the current mean) / sigma
    std::vector<Vector> zs_;       ///< normally distributed vectors used to generate current samples
    std::vector<real> functionValues_; ///< function values evaluated at the current samples
};

} // namespace utils
} // namespace msode

