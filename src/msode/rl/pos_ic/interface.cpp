#include "interface.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvPosIC::EnvPosIC(int maxTries) :
    maxTries_(maxTries)
{}

EnvPosIC::~EnvPosIC() = default;

void EnvPosIC::update(bool /* succesfulTry */)
{}

const std::vector<real3>& EnvPosIC::generateNewPositionsEveryMaxTries(std::mt19937& gen, int n, bool succesfulTry)
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
