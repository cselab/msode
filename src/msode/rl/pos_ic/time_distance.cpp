// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "time_distance.h"

namespace msode {
namespace rl {

EnvPosICTimeDistance::EnvPosICTimeDistance(bool ball, real travelTime, msode::analytic_control::MatrixReal V) :
    ball_(ball),
    travelTime_(travelTime),
    V_(std::move(V))
{}

std::unique_ptr<EnvPosIC> EnvPosICTimeDistance::clone() const
{
    return std::make_unique<EnvPosICTimeDistance>(*this);
}

real3 EnvPosICTimeDistance::getLowestPosition()  const
{
    return - getHighestPosition();
}

real3 EnvPosICTimeDistance::getHighestPosition() const
{
    real dmax{0.0_r};
    for (int i = 0; i < V_.cols(); ++i)
        dmax = std::max(dmax, V_(i,i) * travelTime_);

    return {dmax, dmax, dmax};
}


/// Generate travel time along x, y and z
static real3 generateTravelTimes(std::mt19937& gen, real travelTime)
{
    std::exponential_distribution<real> d(1.0_r);

    real3 T {d(gen), d(gen), d(gen)};
    const real T1 = T.x + T.y + T.z; // norm 1

    T *= travelTime / T1;
    return T;
}

static int randomSign(std::mt19937& gen)
{
    const int x = gen() % 2;
    return 2 * x - 1;
}

static std::vector<real> genPositions1D(std::mt19937& gen, real travelTime, const msode::analytic_control::MatrixReal& V)
{
    std::exponential_distribution<real> d(1.0_r);
    const int n = V.cols();
    std::vector<real> betas(n), x(n, 0.0_r);

    real betas1 {0.0_r};

    for (auto& b : betas)
    {
        b = d(gen) * randomSign(gen);
        betas1 += std::fabs(b);
    }

    for (auto& b : betas)
        b *= travelTime / betas1;

    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            x[i] += V(i,j) * betas[j];

    return x;
}

std::vector<real3> EnvPosICTimeDistance::generateNewPositions(std::mt19937& gen, int n)
{
    return _generatePositions(gen, n, travelTime_);
}

std::vector<real3> EnvPosICTimeDistance::generateUniformPositions(std::mt19937& gen, int n) const
{
    return _generatePositions(gen, n, travelTime_);
}

std::vector<real3> EnvPosICTimeDistance::_generatePositions(std::mt19937& gen, int n, real travelTime) const
{
    MSODE_Expect(n == V_.cols(), "Mismatch in velocity matrix dimensions. Got n=%d instead of %ld.", n, V_.cols());

    if (ball_)
    {
        std::uniform_real_distribution<real> u(2.0_r, travelTime);
        travelTime = u(gen);
    }

    const real3 T = generateTravelTimes(gen, travelTime);
    std::vector<real3> positions(n);

    const auto x = genPositions1D(gen, T.x, V_);
    const auto y = genPositions1D(gen, T.y, V_);
    const auto z = genPositions1D(gen, T.z, V_);

    for (size_t i = 0; i < static_cast<size_t>(n); ++i)
    {
        positions[i] = make_real3(x[i], y[i], z[i]);
    }

    return positions;
}

} // namespace rl
} // namespace msode
