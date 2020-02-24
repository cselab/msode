#include "interface.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvSpace::EnvSpace() = default;
EnvSpace::~EnvSpace() = default;

const std::vector<real3>& EnvSpace::generateNewPositionsIfFlag(std::mt19937& gen, int n, bool generateNew)
{
    if (generateNew)
    {
        savedPositions_ = this->generateNewPositions(gen, n);
        savedPositionsInitialized_ = true;
    }
    else
    {
        MSODE_Ensure(savedPositionsInitialized_, "can not return non initialized saved positions");
    }
    return savedPositions_;
}

} // namespace rl
} // namespace msode
