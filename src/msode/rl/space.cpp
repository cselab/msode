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
        savedPositions_ = this->generateNewPositions(gen, n);
        savedPositionsInitialized_ = true;
    }
    else
    {
        MSODE_Ensure(savedPositionsInitialized_, "can not return non initialized saved positions");
    }
    return savedPositions_;
}

EnvSpaceBox::EnvSpaceBox(real L_) :
    domain_{{-L_, -L_, -L_},
            {+L_, +L_, +L_}}
{}

std::unique_ptr<EnvSpace> EnvSpaceBox::clone() const
{
    return std::make_unique<EnvSpaceBox>(*this);
}

real3 EnvSpaceBox::getLowestPosition()  const {return domain_.lo;}
real3 EnvSpaceBox::getHighestPosition() const {return domain_.hi;}

real EnvSpaceBox::computeMaxDistanceToTarget() const
{
    auto distFromDst = [](real3 dst, real3 r) {return length(r-dst);};
    real d{0.0_r};
    for (auto r : domain_.getCorners())
        d = std::max(d, distFromDst(target, r));
    return d;
}

std::vector<real3> EnvSpaceBox::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    for (auto& p : positions)
        p = utils::generateUniformPositionBox(gen, domain_.lo, domain_.hi); 
    return positions;
}




EnvSpaceBall::EnvSpaceBall(real radius) :
    radius_(radius)
{}

std::unique_ptr<EnvSpace> EnvSpaceBall::clone() const
{
    return std::make_unique<EnvSpaceBall>(*this);
}

real3 EnvSpaceBall::getLowestPosition()  const {return {-radius_, -radius_, -radius_};}
real3 EnvSpaceBall::getHighestPosition() const {return {+radius_, +radius_, +radius_};}

real EnvSpaceBall::computeMaxDistanceToTarget() const {return radius_;}

std::vector<real3> EnvSpaceBall::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    for (auto& p : positions)
        p = utils::generateUniformPositionBall(gen, radius_);
    return positions;
}



EnvSpaceBallCuriculumStateRW::EnvSpaceBallCuriculumStateRW(real radius, real targetRadius, real sigmaRandomWalk) :
    EnvSpaceBall(radius),
    targetRadius_(targetRadius),
    sigmaRandomWalk_(sigmaRandomWalk)
{}


std::unique_ptr<EnvSpace> EnvSpaceBallCuriculumStateRW::clone() const
{
    return std::make_unique<EnvSpaceBallCuriculumStateRW>(*this);
}

static inline real3 generateOnePositionMC(std::mt19937& gen, real3 r0, real radius, real targetRadius, real sigma)
{
    bool accepted {false};
    real3 r;
    std::normal_distribution<real> distr {0.0_r, sigma};

    while (!accepted)
    {
        const real3 step {distr(gen), distr(gen), distr(gen)};
        r = r0 + step;

        if (dot(r,r) <= radius * radius &&
            dot(r,r) > targetRadius * targetRadius)
            accepted = true;
    }
    return r;
}

std::vector<real3> EnvSpaceBallCuriculumStateRW::generateNewPositions(std::mt19937& gen, int n)
{
    std::vector<real3> positions(n);
    
    if (!initialized_)
    {
        constexpr real eps = 0.01_r;
        previousPositions_.resize(n);
        for (auto& p : previousPositions_)
            p = utils::generateUniformPositionShell(gen, targetRadius_, targetRadius_ * (1.0_r + eps));
        initialized_ = true;
    }

    for (int i = 0; i < n; ++i)
        positions[i] = generateOnePositionMC(gen, previousPositions_[i], radius_, targetRadius_, sigmaRandomWalk_);
        
    previousPositions_ = positions;
    return positions;
}



EnvSpaceBallCuriculumActionRW::EnvSpaceBallCuriculumActionRW(std::unique_ptr<MSodeEnvironment>&& environment,
                                                             real radius, real targetRadius, real sigmaRandomWalk) :
    EnvSpaceBall(radius),
    targetRadius_(targetRadius),
    sigmaRandomWalk_(sigmaRandomWalk),
    environment_(std::move(environment))
{}

std::unique_ptr<EnvSpace> EnvSpaceBallCuriculumActionRW::clone() const
{
    return std::make_unique<EnvSpaceBallCuriculumActionRW>(*this);
}


static inline bool isCorrectSample(const std::vector<real3>& positions, real Rmin, real Rmax)
{
    bool allPositionsInsideRmin {true};
    
    for (auto p : positions)
    {
        const auto r = length(p);
        if (r > Rmax)
            return false;
        if (r > Rmin)
            allPositionsInsideRmin = false;
    }
    return ! allPositionsInsideRmin;
}

std::vector<real3> EnvSpaceBallCuriculumActionRW::generateNewPositions(std::mt19937& gen, int n)
{
    if (!initialized_)
    {
        constexpr real eps = 0.01_r;
        previousPositions_.resize(n);
        for (auto& p : previousPositions_)
            p = utils::generateUniformPositionShell(gen, targetRadius_, targetRadius_ * (1.0_r + eps));
        initialized_ = true;
    }

    std::vector<real3> positions;

    do {
        environment_->reset(gen);
        environment_->setPositions(previousPositions_);
        
        const auto action = _generateAction(gen);
        environment_->advance(action);
        
        positions = environment_->getPositions();
    } while(!isCorrectSample(positions, targetRadius_, radius_));
    
    previousPositions_ = positions;
    return positions;
}

std::vector<double> EnvSpaceBallCuriculumActionRW::_generateAction(std::mt19937& gen) const
{
    std::vector<double> actions;
    auto state = environment_->magnFieldState.get();

    if (dynamic_cast<MagnFieldFromActionFromLocalFrame*>(state))
    {
        actions.resize(4);
        const auto& bodies = environment_->getBodies();

        std::uniform_int_distribution<int> bodyIdDistr (0, bodies.size()-1);
        
        const int i = bodyIdDistr(gen);
        const real omega = bodies[i].stepOutFrequency(environment_->fieldMagnitude);

        std::normal_distribution<real> omegaDistr (omega, sigmaRandomWalk_);
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
