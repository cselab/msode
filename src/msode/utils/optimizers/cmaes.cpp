#include "cmaes.h"

namespace msode {
namespace utils {

CMAES::CMAES(const Function& function, int lambda, Vector mean, real sigma, long seed) :
    function_(function),
    gen_(seed),
    lambda_(lambda),
    mu_(lambda/2),
    sigma_(sigma),
    mean_(mean),
    C_     (Matrix::Identity(mean.rows(), mean.rows())),
    pC_    (Vector::Zero(mean.rows())),
    pSigma_(Vector::Zero(mean.rows()))
{
    n_ = mean.rows();

    weights_.resize(mu_);

    for (int i = 0; i < n_; ++i)
    {
        weights_[i] = std::log(std::max( (real)mu_, 0.5_r * lambda_) + 0.5_r) - std::log(i+1.0_r);
    }

    // Normalize weights vector and set muW
    real s1 = 0.0;
    real s2 = 0.0;

    for (int i = 0; i < lambda; ++i)
    {
        s1 += weights_[i];
        s2 += weights_[i]*weights_[i];
    }
    
    for (auto& w : weights_)
        w /= s1;

    muW_ = s1 * s1 / s2;
    
    cC_     = 4.0_r / n_;
    cSigma_ = 4.0_r / n_;
    c1_     = 2.0_r / (n_*n_);
    cMu_    = muW_ / (n_*n_);
    dSigma_ = 1.0_r + std::sqrt(muW_ / n_);

    chiSquareNumber_ = std::sqrt((real) n_) * (1._r - 1._r/(4._r*n_) + 1._r/(21._r*n_*n_));

    order_.resize(lambda_);
    samples_.resize(lambda_);
    ys_.resize(lambda_);
    functionValues_.resize(lambda_);
}

void CMAES::runGeneration()
{
    // sample
    CDecomposition_.compute(C_);

    for (int i = 0; i < lambda_; ++i)
    {
        samples_       [i] = _generateSample();
        functionValues_[i] = function_(samples_[i]);
    }

    _computeOrdering(functionValues_);

    Vector yw = Vector::Zero(n_);

    for (int i = 0; i < mu_; ++i)
    {
        const int id = order_[i];
        yw += weights_[id] * ys_[id];
    }
    
    // update mean

    mean_ += sigma_ * yw;
    
    // cumulation for C

    const real omcc = 1.0_r - cC_;
    pC_ = omcc * pC_;

    if (pSigma_.norm() < 1.5_r * std::sqrt(n_))
        pC_ += std::sqrt((1.0_r - omcc*omcc) * muW_) * yw;
    
    // cumulation for sigma

    const real omcs = 1.0_r - cSigma_;
    pSigma_ = omcs * pSigma_;

    const auto& U = CDecomposition_.eigenvectors().real();
    const auto& D = CDecomposition_.eigenvalues().real();

    // C^{-1/2}
    Matrix Cmhalf =
        U
        * D.unaryExpr([](real d) {return 1.0_r / std::sqrt(d);}).asDiagonal()
        * U.transpose();

    pSigma_ += std::sqrt((1.0_r - omcs*omcs) * muW_) * Cmhalf * yw;
    
    // update C

    C_ = (1.0_r -  c1_ - cMu_) * C_ + c1_ * pC_ * pC_.transpose();

    for (int i = 0; i < mu_; ++i)
    {
        const int id = order_[i];
        C_ += (cMu_ * weights_[id]) * ys_[id] * ys_[id].transpose();
    }
    
    // update of sigma

    const real argexp = cSigma_ / dSigma_ * (pSigma_.norm() / chiSquareNumber_ - 1.0_r);
    sigma_ *= std::exp(argexp);
}


CMAES::Vector CMAES::_generateSample()
{
    Vector y = Vector::Zero(n_);
    const auto& eigenValues  = CDecomposition_.eigenvalues();
    const auto& eigenVectors = CDecomposition_.eigenvectors();
    
    for (int i = 0; i < n_; ++i)
    {
        const real li = eigenValues(i).real();
        std::normal_distribution<real> distr(0.0_r, std::sqrt(li));
        y(i) = distr(gen_);
    }

    return mean_ + sigma_ * (eigenVectors.real() * y);
}


void CMAES::_computeOrdering(const std::vector<real>& values)
{
    std::iota(order_.begin(), order_.end(), 0);
    std::sort(order_.begin(), order_.end(),
              [&](int a, int b) { return values[a] > values[b]; } );
}


} // namespace utils
} // namespace msode

