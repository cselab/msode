#include "interface.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvSpace::EnvSpace(int maxTries) :
    maxTries_(maxTries)
{}

EnvSpace::~EnvSpace() = default;

const std::vector<real3>& EnvSpace::generateNewPositionsEveryMaxTries(std::mt19937& gen, int n, bool succesfulTry)
{
    if (!savedPositionsInitialized_)
    {
        savedPositions_ = this->generateNewPositions(gen, n);
        savedPositionsInitialized_ = true;
        return savedPositions_;
    }
    else
    {
        if (succesfulTry || numTries_ >= maxTries_)
        {
            savedPositions_ = this->generateNewPositions(gen, n);
            numTries_ = 0;
        }
        else
        {
            ++numTries_;
        }
    }
    return savedPositions_;
}

} // namespace rl
} // namespace msode
