// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "time_distance.h"

#include <msode/analytic_control/helpers.h>
#include <msode/utils/curriculum_counter.h>

namespace msode {
namespace rl {

/** Draw position randomly with a specified travel time.
    The travel time increases linearly after each success, in a curriculum manner.
 */
class EnvPosICTimeDistanceCurriculum: public EnvPosICTimeDistance
{
public:
    EnvPosICTimeDistanceCurriculum(bool ball,
                                   real initialTravelTime,
                                   real maxTravelTime,
                                   real successIncrement,
                                   msode::analytic_control::MatrixReal V,
                                   int numTriesBeforeUpdate,
                                   int requiredSuccesfulTries);

    std::unique_ptr<EnvPosIC> clone() const override;

    real3 getLowestPosition() const override;
    real3 getHighestPosition() const override;

    void update(bool successfulTry) override;

    std::vector<real3> generateUniformPositions(std::mt19937& gen, int n) const override;

private:
    real successIncrement_;
    real maxTravelTime_;
    utils::CurriculumCounter curriculumCounter_;
};

} // namespace rl
} // namespace msode
