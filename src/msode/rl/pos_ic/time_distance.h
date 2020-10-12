// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

#include <msode/analytic_control/helpers.h>

namespace msode {
namespace rl {

/** Draw position randomly with a specified travel time.
 */
class EnvPosICTimeDistance: public EnvPosIC
{
public:
    EnvPosICTimeDistance(bool ball, real travelTime, msode::analytic_control::MatrixReal V);

    std::unique_ptr<EnvPosIC> clone() const override;

    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;
    std::vector<real3> generateUniformPositions(std::mt19937& gen, int n) const override;

protected:
    std::vector<real3> _generatePositions(std::mt19937& gen, int n, real travelTime) const;

protected:
    bool ball_; ///< if true, will generate random travel times up to travelTime_; otherwise will be exactly travelTime_.
    real travelTime_;
    msode::analytic_control::MatrixReal V_; ///< velocity matrix
};

} // namespace rl
} // namespace msode
