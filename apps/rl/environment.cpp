#include "environment.h"

Params::Params(TimeParams time_, RewardParams reward_, real fieldMagnitude_, real distanceThreshold_, std::unique_ptr<EnvSpace>&& space_) :
    time(time_),
    reward(reward_),
    fieldMagnitude(fieldMagnitude_),
    distanceThreshold(distanceThreshold_),
    space(std::move(space_))
{}

