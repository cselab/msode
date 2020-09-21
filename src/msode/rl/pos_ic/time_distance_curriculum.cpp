// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "time_distance_curriculum.h"

namespace msode {
namespace rl {

EnvPosICTimeDistanceCurriculum::EnvPosICTimeDistanceCurriculum(real initialTravelTime,
                                                               real maxTravelTime,
                                                               real successIncrement,
                                                               msode::analytic_control::MatrixReal V) :
    EnvPosICTimeDistance(initialTravelTime, std::move(V)),
    successIncrement_(successIncrement),
    maxTravelTime_(maxTravelTime)
{}

std::unique_ptr<EnvPosIC> EnvPosICTimeDistanceCurriculum::clone() const
{
    return std::make_unique<EnvPosICTimeDistanceCurriculum>(*this);
}

real3 EnvPosICTimeDistanceCurriculum::getHighestPosition() const
{
    real dmax{0.0_r};
    for (int i = 0; i < V_.cols(); ++i)
        dmax = std::max(dmax, V_(i,i) * maxTravelTime_);

    return {dmax, dmax, dmax};
}

void EnvPosICTimeDistanceCurriculum::update(bool successfulTry)
{
    if (!successfulTry)
        return;

    travelTime_ = std::min(travelTime_ + successIncrement_, maxTravelTime_);
}

} // namespace rl
} // namespace msode
