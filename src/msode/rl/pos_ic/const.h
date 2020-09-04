// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

#include <vector>

namespace msode {
namespace rl {

/** Constant positions
 */
class EnvPosICConst : public EnvPosIC
{
public:
    EnvPosICConst(const std::vector<real3>& positions);

    std::unique_ptr<EnvPosIC> clone() const override;

    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;
    std::vector<real3> generateUniformPositions(std::mt19937& gen, int n) const override;

private:
    const std::vector<real3> positions_;
};

} // namespace rl
} // namespace msode
