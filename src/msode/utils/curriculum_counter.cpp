// Copyright 2020 ETH Zurich. All Rights Reserved.

#include "curriculum_counter.h"

namespace msode {
namespace utils {

CurriculumCounter::CurriculumCounter(int numTriesBeforeUpdate,
                                     int requiredSuccesfulTries) :
    maxTries_{numTriesBeforeUpdate},
    requiredSuccesfulTries_{requiredSuccesfulTries}
{}

bool CurriculumCounter::needUpdate(bool successfulTry)
{
    if (successfulTry)
        ++successfulTries_;
    ++numTries_;

    if (successfulTries_ >= requiredSuccesfulTries_)
    {
        numTries_ = 0;
        successfulTries_ = 0;
        return true;
    }

    if (maxTries_ > 0 &&
        numTries_ >= maxTries_)
    {
        numTries_ = 0;
        successfulTries_ = 0;
        return true;
    }

    return false;
}

} // namespace utils
} // namespace msode
