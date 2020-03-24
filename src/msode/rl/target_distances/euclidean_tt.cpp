#include "euclidean_tt.h"

#include <msode/analytic_control/optimal_path.h>

namespace msode {
namespace rl {

TargetDistanceEuclideanTT::TargetDistanceEuclideanTT(real magneticFieldMagnitude) :
    magneticFieldMagnitude_(magneticFieldMagnitude)
{}

static inline std::vector<real3> getPositions(const std::vector<RigidBody>& bodies)
{
    std::vector<real3> positions;
    for (auto b : bodies)
        positions.push_back(b.r);
    return positions;
}



real TargetDistanceEuclideanTT::compute(const std::vector<RigidBody>& bodies) const
{
    if (!initialized_)
    {
        auto V = msode::analytic_control::createVelocityMatrix(magneticFieldMagnitude_, bodies);
        U_ = V.inverse();
        initialized_ = true;
    }

    const auto A = msode::analytic_control::computeA(U_, getPositions(bodies));

    real sum {0.0_r};

    for (auto a : A)
        sum += dot(a, a);
    
    return std::sqrt(sum);
}

} // namespace rl
} // namespace msode
