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

class CMAES
{
public:
    using Vector = Eigen::Matrix<real, Eigen::Dynamic, 1>;
    using Matrix = Eigen::Matrix<real, Eigen::Dynamic, Eigen::Dynamic>;

    using Function = std::function<real(Vector)>;

    CMAES(const Function& function, int lambda, Vector mean, real sigma, long seed);

    void runGeneration();

private:
    Vector _generateSample();
    void _computeOrdering(const std::vector<real>& values);
    
private:
    Function function_;
    std::mt19937 gen_;

    int n_;
    
    int lambda_;
    int mu_;
    real sigma_;
    Vector mean_;
    Matrix C_;
    Vector pC_;
    Vector pSigma_;

    real cC_;
    real cSigma_;
    real c1_;
    real cMu_;
    real dSigma_;
    real chiSquareNumber_;
    std::vector<real> weights_;

    Eigen::EigenSolver<Matrix> CDecomposition_;

    real muW_;

    std::vector<int> order_;
    std::vector<Vector> samples_;
    std::vector<Vector> ys_;
    std::vector<real> functionValues_;
};

} // namespace utils
} // namespace msode

