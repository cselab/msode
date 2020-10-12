// Copyright 2020 ETH Zurich. All Rights Reserved.
#include "time_distance_curriculum.h"

namespace msode {
namespace rl {

EnvPosICTimeDistanceCurriculum::EnvPosICTimeDistanceCurriculum(bool ball,
                                                               real initialTravelTime,
                                                               real maxTravelTime,
                                                               real successIncrement,
                                                               msode::analytic_control::MatrixReal V,
                                                               int numTriesBeforeUpdate,
                                                               int requiredSuccesfulTries) :
    EnvPosICTimeDistance(ball, initialTravelTime, std::move(V)),
    successIncrement_(successIncrement),
    maxTravelTime_(maxTravelTime),
    curriculumCounter_(numTriesBeforeUpdate, requiredSuccesfulTries)
{}

std::unique_ptr<EnvPosIC> EnvPosICTimeDistanceCurriculum::clone() const
{
    return std::make_unique<EnvPosICTimeDistanceCurriculum>(*this);
}

real3 EnvPosICTimeDistanceCurriculum::getLowestPosition() const
{
    return - getHighestPosition();
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
    if (curriculumCounter_.needUpdate(successfulTry))
        travelTime_ = std::min(travelTime_ + successIncrement_, maxTravelTime_);
}

std::vector<real3> EnvPosICTimeDistanceCurriculum::generateUniformPositions(std::mt19937& gen, int n) const
{
    return _generatePositions(gen, n, maxTravelTime_);
}

} // namespace rl
} // namespace msode
