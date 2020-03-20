#include "cmaes.h"

#include <msode/core/log.h>

#include <limits>

namespace msode {
namespace utils {

static inline real safeSqrt(real x)
{
    MSODE_Expect(x >= 0, "bad value for sqrt");
    return std::sqrt(x);
}

CMAES::CMAES(const Function& function, int lambda, Vector mean, real sigma, long seed) :
    function_(function),
    gen_(seed),
    lambda_(lambda),
    mu_(lambda/2),
    sigma_(sigma),
    xmean_(mean),
    C_     (Matrix::Identity(mean.rows(), mean.rows())),
    pC_    (Vector::Zero(mean.rows())),
    pSigma_(Vector::Zero(mean.rows()))
{
    n_ = mean.rows();

    weights_.resize(mu_);

    for (int i = 0; i < mu_; ++i)
    {
        weights_[i] = std::log(std::max( (real)mu_, 0.5_r * lambda_) + 0.5_r) - std::log(i+1.0_r);
    }

    // Normalize weights vector and set muW
    real s1 = 0.0;
    real s2 = 0.0;

    for (int i = 0; i < mu_; ++i)
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
    dSigma_ = 1.0_r + 2.0_r * std::max(0.0_r, safeSqrt((muEff_-1.0_r)/(n_+1.0_r))-1.0_r) + cSigma_;

    chiSquareNumber_ = safeSqrt((real) n_) * (1._r - 1._r/(4._r*n_) + 1._r/(21._r*n_*n_));

    order_.resize(lambda_);
    samples_.resize(lambda_);
    ys_.resize(lambda_);
    zs_.resize(lambda_);
    functionValues_.resize(lambda_);

    bestEverValue_     = std::numeric_limits<real>::max();
    previousBestValue_ = std::numeric_limits<real>::max();
    currentBestValue_  = std::numeric_limits<real>::max();
}


std::tuple<CMAES::Vector, real> CMAES::runMinimization(real absoluteThreshold, int maxGeneration, bool verbose)
{
    for (int generation = 0; generation < maxGeneration; ++generation)
    {
        previousBestValue_ = currentBestValue_;
        _runGeneration();
        currentBestValue_ = functionValues_[order_.front()];
        
        if (currentBestValue_ < bestEverValue_)
        {
            bestEverValue_ = currentBestValue_;
            bestEverX_     = samples_[order_.front()];
        }

        if (verbose)
        {
            std::cout << "--------------------------------\n"
                      << "generation " << generation << "\n"
                      << "best value : " << currentBestValue_ << "\n"
                      << bestEverX_.transpose() << "\n"
                      << "---------------------------------" << std::endl;
        }

        if (std::abs(currentBestValue_ - previousBestValue_) < absoluteThreshold)
            break;
    }

    return {std::move(bestEverX_), bestEverValue_};
}

void CMAES::_runGeneration()
{
    // eigen decomposition C = (B D) (D B)'
    CDecomposition_.compute(C_);
    const Matrix B = CDecomposition_.eigenvectors().real();
    Vector D = CDecomposition_.eigenvalues().real();

    for (auto& d : D)
        d = safeSqrt(d);

    // sample
    for (int i = 0; i < lambda_; ++i)
    {
        zs_            [i] = _generateNormalDistrVector();
        ys_            [i] = B * D.asDiagonal() * zs_[i];
        samples_       [i] = xmean_ + sigma_ * ys_[i];
        functionValues_[i] = function_(samples_[i]);
        ++countEval_;
    }

    _computeOrdering(functionValues_);

    Vector zmean = Vector::Zero(n_);

    for (int i = 0; i < mu_; ++i)
    {
        const int id = order_[i];
        zmean += weights_[i] * zs_[id];
    }
    
    // update mean

    xmean_ += sigma_ * B * D.asDiagonal() * zmean;

    // cumulation for sigma

    pSigma_ = (1.0_r-cSigma_) * pSigma_ + (safeSqrt(cSigma_*(2.0_r-cSigma_)*muEff_)) * (B * zmean);

    // cumulation for C

    const real pSigmaNorm = pSigma_.norm();
    
    const int hsig =
        (pSigmaNorm / safeSqrt(1.0_r - std::pow(1.0_r-cSigma_, 2 * (1 + countEval_/lambda_)))/chiSquareNumber_)
        < (1.4_r + 2.0_r /(n_+1));

    pC_ = (1.0_r - cC_) * pC_ + hsig * safeSqrt(cC_ * (2.0_r - cC_) * muEff_) * (B * D.asDiagonal() * zmean);
    
    // adapt covariance matrix C

    C_ = (1.0_r - c1_ - cMu_) * C_ +   // discount old matrix
          c1_ * (pC_ * pC_.transpose() + // rank 1 update
                 (1-hsig) * cC_ * (2.0_r - cC_) * C_);

    for (int i = 0; i < mu_; ++i)
    {
        const int id = order_[i];
        C_ += (cMu_ * weights_[i]) * ys_[id] * ys_[id].transpose();
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

