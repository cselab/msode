#include "ball_curriculum_action.h"

#include <msode/core/math.h>
#include <msode/rl/environment.h>
#include <msode/utils/rnd.h>

#include <msode/rl/field_from_action/local_frame.h>

namespace msode {
namespace rl {

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

    if (dynamic_cast<FieldFromActionFromLocalFrame*>(state))
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
        msode_die("Not implemented with field from action other than FieldFromActionFromLocalFrame\n");
    }
    return actions;
}

} // namespace rl
} // namespace msode
