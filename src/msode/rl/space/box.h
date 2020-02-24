#pragma once

#include "interface.h"

namespace msode {
namespace rl {

class EnvSpaceBox : public EnvSpace
{
public:
    EnvSpaceBox(real L_);

    std::unique_ptr<EnvSpace> clone() const override;
    
    real3 getLowestPosition()  const override;
    real3 getHighestPosition() const override;
    real computeMaxDistanceToTarget() const override;

    std::vector<real3> generateNewPositions(std::mt19937& gen, int n) override;

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
