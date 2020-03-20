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

    muEff_ = s1 * s1 / s2;
    
    cC_     = (4.0_r+ muEff_ / n_) / (n_ + 4.0_r + 2.0_r * muEff_ / n_);
    cSigma_ = (muEff_ + 2.0_r)/(n_ + muEff_ + 5.0_r);
    c1_     = 2.0_r / (std::pow(n_ + 1.3_r, 2) + muEff_);
    cMu_    = 2.0_r * (muEff_ - 2.0_r + 1.0_r / muEff_) / (std::pow(n_ + 2, 2) + muEff_); 
    dSigma_ = 1.0_r + 2.0_r * std::max(0.0_r, std::sqrt((muEff_-1.0_r)/(n_+1.0_r))-1.0_r) + cSigma_;

    chiSquareNumber_ = std::sqrt((real) n_) * (1._r - 1._r/(4._r*n_) + 1._r/(21._r*n_*n_));

    order_.resize(lambda_);
    samples_.resize(lambda_);
    ys_.resize(lambda_);
    zs_.resize(lambda_);
    functionValues_.resize(lambda_);
}

void CMAES::runGeneration()
{
    // eigen decomposition C = (B D) (D B)'
    CDecomposition_.compute(C_);
    const Matrix B = CDecomposition_.eigenvectors().real();
    Vector D = CDecomposition_.eigenvalues().real();

    for (auto& d : D)
        d = std::sqrt(d);

    // std::cout << "B = " << B << std::endl;
    // std::cout << "D = " << D << std::endl;
    // std::cout << "C = " << C_ << std::endl;

    // sample
    for (int i = 0; i < lambda_; ++i)
    {
        zs_            [i] = _generateNormalDistrVector();
        ys_            [i] = B * D.asDiagonal() * zs_[i];
        // std::cout << i << " -> " << ys_[i].transpose() << std::endl;
        samples_       [i] = mean_ + sigma_ * ys_[i];
        functionValues_[i] = function_(samples_[i]);
    }

    _computeOrdering(functionValues_);

    Vector zmean = Vector::Zero(n_);

    for (int i = 0; i < mu_; ++i)
    {
        const int id = order_[i];
        zmean += weights_[id] * zs_[id];
    }

    {
        const int id = order_.front();
        const auto& s = samples_[id];
        const auto f = functionValues_[id];
        printf("f = %g  [%g %g %g], sigma = %g\n",
               f, s(0), s(1), s(2), sigma_);
    }
    
    // update mean

    mean_ += sigma_ * B * D.asDiagonal() * zmean;

    // cumulation for sigma

    pSigma_ = (1.0_r-cSigma_) * pSigma_ + (std::sqrt(cSigma_*(2.0_r-cSigma_)*muEff_)) * (B * zmean);

    // cumulation for C

    const real pSigmaNorm = pSigma_.norm();
    
    const int hsig =
        (pSigmaNorm / std::sqrt(1.0_r - std::pow(1.0_r-cSigma_,2*countEval_/lambda_))/chiSquareNumber_)
        < (1.4_r + 2.0_r /(n_+1));

    pC_ = (1.0_r - cC_) * pC_ + hsig * std::sqrt(cC_ * (2.0_r -cC_) * muEff_) * (B * D.asDiagonal() * zmean);
    
    // adapt covariance matrix C

    C_ = (1.0_r - c1_ - cMu_) * C_ +   // discount old matrix
          c1_ * (pC_ * pC_.transpose() + // rank 1 update
                 (1-hsig) * cC_ * (2.0_r - cC_) * C_);

    for (int i = 0; i < mu_; ++i)
    {
        const int id = order_[i];
        C_ += (cMu_ * weights_[id]) * ys_[id] * ys_[id].transpose();
    }

    // adapt step size sigma

    const real argexp = cSigma_ / dSigma_ * (pSigmaNorm / chiSquareNumber_ - 1.0_r);
    sigma_ *= std::exp(argexp);
}


CMAES::Vector CMAES::_generateNormalDistrVector()
{
    Vector z = Vector::Zero(n_);

    for (auto& val : z)
        val = normDistr_(gen_);

    return z;
}


void CMAES::_computeOrdering(const std::vector<real>& values)
{
    std::iota(order_.begin(), order_.end(), 0);
    std::sort(order_.begin(), order_.end(),
              [&](int a, int b) { return values[a] < values[b]; } );
}


} // namespace utils
} // namespace msode

