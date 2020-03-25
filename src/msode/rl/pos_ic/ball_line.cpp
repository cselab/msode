#include "ball_line.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

EnvPosICBallLine::EnvPosICBallLine(real radius, real probLine) :
    EnvPosICBall(radius),
    probLine_(probLine)
{}

std::unique_ptr<EnvPosIC> EnvPosICBallLine::clone() const
{
    return std::make_unique<EnvPosICBallLine>(*this);
}

std::vector<real3> EnvPosICBallLine::generateNewPositions(std::mt19937& gen, int n)
{
    std::uniform_real_distribution<real> U(0.0_r, 1.0_r);

    if (U(gen) < probLine_)
        return _generateLines(gen, n);
    return generateUniformPositions(gen, n);
}


static inline real3 makeRandomUnitVector(std::mt19937& gen)
{
     std::uniform_real_distribution<real> U(0.0_r, 1.0_r);
     const real theta = 2.0_r * M_PI * U(gen);
     const real phi   = std::acos(1.0_r - 2.0_r * U(gen));

     return {std::sin(phi) * std::cos(theta),
             std::sin(phi) * std::sin(theta),
             std::cos(phi)};
}

std::vector<real3> EnvPosICBallLine::_generateLines(std::mt19937& gen, int n) const
{
    std::vector<real3> positions(n);

    std::uniform_real_distribution<real> Udistance(-radius_, radius_);

    const real3 direction = makeRandomUnitVector(gen);

    for (auto& p : positions)
        p = Udistance(gen) * direction;

    return positions;
}

} // namespace rl
} // namespace msode
