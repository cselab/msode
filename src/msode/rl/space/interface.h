#pragma once

#include <msode/core/types.h>

#include <memory>
#include <random>
#include <vector>

namespace msode {
namespace rl {

class EnvSpace
{
public:
    EnvSpace();
    virtual ~EnvSpace();

    virtual std::unique_ptr<EnvSpace> clone() const = 0;
    
    virtual real3 getLowestPosition()  const = 0;
    virtual real3 getHighestPosition() const = 0;
    virtual real computeMaxDistanceToTarget() const = 0;

    const std::vector<real3>& generateNewPositionsIfFlag(std::mt19937& gen, int n, bool generateNew);
    virtual std::vector<real3> generateNewPositions(std::mt19937& gen, int n) = 0;

public:
    const real3 target {0.0_r, 0.0_r, 0.0_r};

private:
    bool savedPositionsInitialized_ {false};
    std::vector<real3> savedPositions_;
};

} // namespace rl
} // namespace msode
