// Copyright 2020 ETH Zurich. All Rights Reserved.
#pragma once

namespace msode {
namespace utils {

/** Counter to manage the update of a curriculum-driven
    initial position (IC) module.

    The IC is updated if one of the following occur:
    - The number of successes since the last update is greater than a given value.
    - The number of attempts since the last update is greater than a given value.
 */
class Curriculum
{
public:

    /**
       \param numTriesBeforeUpdate The number of tries (successful or not) before forcing an update.
       Non positive values will make this equivalent to +infty.
       \param requiredSuccesfulTries The number of successful tries after which to force and update.
     */
    Curriculum(int numTriesBeforeUpdate,
               int requiredSuccesfulTries);

    bool needUpdate(bool successfulTry);

private:
    int numTries_ {0};
    int successfulTries_ {0};

    int maxTries_;
    int requiredSuccesfulTries_;
};

} // namespace utils
} // namespace msode
