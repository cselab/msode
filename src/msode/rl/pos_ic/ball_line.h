// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "ball.h"

namespace msode {
namespace rl {

class EnvPosICBallLine : public EnvPosICBall
{
public:
    EnvPosICBallLine(real radius, real probLine);

    std::unique_ptr<EnvPosIC> clone() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

private:
    std::vector<real3> _generateLines(std::mt19937& gen, int n) const;

protected:
    const real probLine_; ///< the probability to select all swimmers on a random line
};

} // namespace rl
} // namespace msode
