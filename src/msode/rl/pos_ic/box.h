// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

#include "interface.h"

#include <array>

namespace msode {
namespace rl {

class EnvPosICBox : public EnvPosIC
{
public:
    EnvPosICBox(real L);

    std::unique_ptr<EnvPosIC> clone() const override;

    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;
    std::vector<real3> generateUniformPositions(std::mt19937& gen, int n) const override;

private:
    struct Box
    {
        real3 lo, hi;
        std::array<real3, 8> getCorners() const
        {
            return {real3 {lo.x, lo.y, lo.z},
                    real3 {lo.x, lo.y, hi.z},
                    real3 {lo.x, hi.y, lo.z},
                    real3 {lo.x, hi.y, hi.z},
                    real3 {hi.x, lo.y, lo.z},
                    real3 {hi.x, lo.y, hi.z},
                    real3 {hi.x, hi.y, lo.z},
                    real3 {hi.x, hi.y, hi.z}};
        }
    };

    const Box domain_;
};

} // namespace rl
} // namespace msode
