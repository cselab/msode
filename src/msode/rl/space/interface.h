#pragma once

#include <msode/core/types.h>

#include <memory>
#include <random>
#include <vector>

namespace msode {
namespace rl {

/** Class to draw an initial position randomly in the state space
 */
class EnvSpace
{
public:
    EnvSpace(int maxTries);
    virtual ~EnvSpace();

    virtual std::unique_ptr<EnvSpace> clone() const = 0;
    
    virtual real3 getLowestPosition()  const = 0;
    virtual real3 getHighestPosition() const = 0;
    virtual real computeMaxDistanceToTarget() const = 0;

    const std::vector<real3>& generateNewPositionsEveryMaxTries(std::mt19937& gen, int n, bool succesfulTry);
    virtual std::vector<real3> generateNewPositions(std::mt19937& gen, int n) = 0;

public:
    const real3 target {0.0_r, 0.0_r, 0.0_r};

private:
    int numTries_ {0};
    const int maxTries_;
    bool savedPositionsInitialized_ {false};
    std::vector<real3> savedPositions_;
};

} // namespace rl
} // namespace msode
