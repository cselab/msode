#include <msode/utils/curriculum_counter.h>

#include <gtest/gtest.h>
#include <cmath>

using namespace msode;


GTEST_TEST(curriculum, update_max_tries)
{
    const bool success = false;
    const int numTriesBeforeUpdate = 5;
    const int requiredSuccesfulTries = 99999999;
    utils::CurriculumCounter curriculumCounter(numTriesBeforeUpdate, requiredSuccesfulTries);

    const int numTries = 1000;
    int numUpdates = 0;
    for (int i = 0; i < numTries; ++i)
    {
        const bool needUpdate = curriculumCounter.needUpdate(success);
        if (needUpdate)
            ++numUpdates;
    }

    ASSERT_EQ(numUpdates, numTries / numTriesBeforeUpdate);
}

GTEST_TEST(curriculum, update_successes)
{
    const bool success = true;
    const int numTriesBeforeUpdate = 0; // infty
    const int requiredSuccesfulTries = 42;
    utils::CurriculumCounter curriculumCounter(numTriesBeforeUpdate, requiredSuccesfulTries);

    const int numTries = 1000;
    int numUpdates = 0;
    for (int i = 0; i < numTries; ++i)
    {
        const bool needUpdate = curriculumCounter.needUpdate(success);
        if (needUpdate)
            ++numUpdates;
    }

    ASSERT_EQ(numUpdates, numTries / requiredSuccesfulTries);
}

GTEST_TEST(curriculum, no_update)
{
    const bool success = false;
    const int numTriesBeforeUpdate = 0; // infty
    const int requiredSuccesfulTries = 42;
    utils::CurriculumCounter curriculumCounter(numTriesBeforeUpdate, requiredSuccesfulTries);

    const int numTries = 1000;
    int numUpdates = 0;
    for (int i = 0; i < numTries; ++i)
    {
        const bool needUpdate = curriculumCounter.needUpdate(success);
        if (needUpdate)
            ++numUpdates;
    }

    ASSERT_EQ(numUpdates, 0);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
