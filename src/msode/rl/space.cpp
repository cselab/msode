#include "space.h"
#include "environment.h"

#include <msode/core/math.h>
#include <msode/utils/rnd.h>

namespace msode {
namespace rl {

const std::vector<real3>& EnvSpace::generateNewPositionsIfFlag(std::mt19937& gen, int n, bool generateNew)
{
    if (generateNew)
    {
        savedPositions = this->generateNewPositions(gen, n);
        savedPositionsInitialized = true;
    }
    else
    {
        MSODE_Ensure(savedPositionsInitialized, "can not return non initialized saved positions");
    }
    return savedPositions;
}

EnvSpaceBox::EnvSpaceBox(real L_) :
    domain{{-L_, -L_, -L_},
           {+L_, +L_, +L_}}
{}

std::unique_ptr<EnvSpace> EnvSpaceBox::clone() const
{
    return std::make_unique<EnvSpaceBox>(*this);
}

real3 EnvSpaceBox::getLowestPosition()  const {return domain.lo;}
real3 EnvSpaceBox::getHighestPosition() const {return domain.hi;}

real EnvSpaceBox::computeMaxDistanceToTarget() const
{
    auto distFromDst = [](real3 dst, real3 r) {return length(r-dst);};
    real d{0.0_r};
    for (auto r : domain.getCorners())
        d = std::max(d, distFromDst(target, r));
    return d;
}

std::vector<real3> EnvSpaceBox::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    for (auto& p : positions)
        p = utils::generateUniformPositionBox(gen, domain.lo, domain.hi); 
    return positions;
}




EnvSpaceBall::EnvSpaceBall(real radius_) :
    radius(radius_)
{}

std::unique_ptr<EnvSpace> EnvSpaceBall::clone() const
{
    return std::make_unique<EnvSpaceBall>(*this);
}

real3 EnvSpaceBall::getLowestPosition()  const {return {-radius, -radius, -radius};}
real3 EnvSpaceBall::getHighestPosition() const {return {+radius, +radius, +radius};}

real EnvSpaceBall::computeMaxDistanceToTarget() const {return radius;}

std::vector<real3> EnvSpaceBall::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    for (auto& p : positions)
        p = utils::generateUniformPositionBall(gen, radius);
    return positions;
}



EnvSpaceBallCuriculumStateRW::EnvSpaceBallCuriculumStateRW(real radius_, real targetRadius_, real sigmaRandomWalk_) :
    EnvSpaceBall(radius_),
    targetRadius(targetRadius_),
    sigmaRandomWalk(sigmaRandomWalk_)
{}


std::unique_ptr<EnvSpace> EnvSpaceBallCuriculumStateRW::clone() const
{
    return std::make_unique<EnvSpaceBallCuriculumStateRW>(*this);
}

static inline real3 generateOnePositionMC(std::mt19937& gen, real3 r0, real radius, real sigma)
{
    bool accepted {false};
    real3 r;
    std::normal_distribution<real> distr {0.0_r, sigma};

    while (!accepted)
    {
        const real3 step {distr(gen), distr(gen), distr(gen)};
        r = r0 + step;

        if (dot(r,r) <= radius * radius)
            accepted = true;
    }
    return r;
}

std::vector<real3> EnvSpaceBallCuriculumStateRW::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    
    if (!initialized)
    {
        previousPositions.resize(n);
        for (auto& p : previousPositions)
            p = utils::generateUniformPositionBall(gen, targetRadius);
        initialized = true;
    }

    for (int i = 0; i < n; ++i)
        positions[i] = generateOnePositionMC(gen, previousPositions[i], radius, sigmaRandomWalk);
        
    previousPositions = positions;
    return positions;
}



EnvSpaceBallCuriculumActionRW::EnvSpaceBallCuriculumActionRW(std::unique_ptr<MSodeEnvironment>&& environment_,
                                                             real radius_, real targetRadius_, real sigmaRandomWalk_) :
    EnvSpaceBall(radius_),
    targetRadius(targetRadius_),
    sigmaRandomWalk(sigmaRandomWalk_),
    environment(std::move(environment_))
{}

std::unique_ptr<EnvSpace> EnvSpaceBallCuriculumActionRW::clone() const
{
    return std::make_unique<EnvSpaceBallCuriculumActionRW>(*this);
}


static inline bool isCorrectSample(const std::vector<real3>& positions, real Rmin, real Rmax)
{
    for (auto p : positions)
    {
        const auto r = length(p);
        if (r < Rmin || r > Rmax)
            return false;
    }
    return true;
}

std::vector<real3> EnvSpaceBallCuriculumActionRW::generateNewPositions(std::mt19937& gen, int n)
{
    if (!initialized)
    {
        previousPositions.resize(n);
        for (auto& p : previousPositions)
            p = utils::generateUniformPositionBall(gen, targetRadius);
        initialized = true;
    }

    std::vector<real3> positions;

    do {
        environment->reset(gen);
        environment->setPositions(previousPositions);
        
        const auto action = generateAction(gen);
        environment->advance(action);
        
        positions = environment->getPositions();
    } while(!isCorrectSample(positions, targetRadius, radius));

    auto r = positions[0];
    auto rp = previousPositions[0];
    printf("%.6g %.6g %.6g     %.6g %.6g %.6g\n",
           r.x, r.y, r.z, rp.x, rp.y, rp.z);
    
    previousPositions = positions;
    return positions;
}

std::vector<double> EnvSpaceBallCuriculumActionRW::generateAction(std::mt19937& gen) const
{
    std::vector<double> actions;
    auto state = environment->magnFieldState.get();

    if (dynamic_cast<MagnFieldFromActionFromLocalFrame*>(state))
    {
        actions.resize(4);
        const auto& bodies = environment->getBodies();

        std::uniform_int_distribution<int> bodyIdDistr (0, bodies.size()-1);
        
        const int i = bodyIdDistr(gen);
        const real omega = bodies[i].stepOutFrequency(environment->fieldMagnitude);

        std::normal_distribution<real> omegaDistr (omega, sigmaRandomWalk);
        std::uniform_real_distribution<real> udistr (-1.0_r, 1.0_r);

        actions[0] = omegaDistr(gen);
        actions[1] = udistr(gen);
        actions[2] = udistr(gen);
        actions[3] = udistr(gen);
    }
    else
    {
        msode_die("Not implemented with field from action other than MagnFieldFromActionFromLocalFrame\n");
    }
    return actions;
}

} // namespace rl
} // namespace msode
