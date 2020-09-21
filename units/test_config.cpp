#include <msode/core/config.h>

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

using namespace msode;

GTEST_TEST( CONFIG, propulsion_matrix )
{
    PropulsionMatrix p1;
    p1.A[0] = 1.0_r;
    p1.A[1] = 2.0_r;
    p1.A[2] = 3.0_r;

    p1.B[0] = 4.0_r;
    p1.B[1] = 5.0_r;
    p1.B[2] = 6.0_r;

    p1.C[0] = 7.0_r;
    p1.C[1] = 8.0_r;
    p1.C[2] = 9.0_r;

    // prop matrix to config
    const Config config = p1;

    // config to prop matrix
    const auto p2 = config.get<PropulsionMatrix>();

    for (int i = 0; i < 3; ++i)
    {
        ASSERT_EQ(p1.A[i], p2.A[i]);
        ASSERT_EQ(p1.B[i], p2.B[i]);
        ASSERT_EQ(p1.C[i], p2.C[i]);
    }
}

GTEST_TEST( CONFIG, str2quaternion )
{
    Config config = json::parse(R"(
    {
        "q" : [1, 2, 3, 4]
    })");

    const auto q = config.at("q").get<Quaternion>();

    ASSERT_EQ(q.w, 1.0_r);
    ASSERT_EQ(q.x, 2.0_r);
    ASSERT_EQ(q.y, 3.0_r);
    ASSERT_EQ(q.z, 4.0_r);
}

GTEST_TEST( CONFIG, quaternion )
{
    const auto q1 = Quaternion::createFromComponents(0.1_r, 0.2_r, 0.3_r, 0.4_r);

    // quaternion to config
    const Config config = q1;

    // config to quaternion
    const auto q2 = config.get<Quaternion>();

    ASSERT_EQ(q1.w, q2.w);
    ASSERT_EQ(q1.x, q2.x);
    ASSERT_EQ(q1.y, q2.y);
    ASSERT_EQ(q1.z, q2.z);
}

GTEST_TEST( CONFIG, real3 )
{
    const real3 r1 = {0.1_r, 0.2_r, 0.3_r};
    const Config config = r1;
    const auto r2 = config.get<real3>();

    ASSERT_EQ(r1.x, r2.x);
    ASSERT_EQ(r1.y, r2.y);
    ASSERT_EQ(r1.z, r2.z);
}

GTEST_TEST( CONFIG, pointer )
{
    Config config = json::parse(R"(
    {
        "main" : {
            "a" : 1,
            "b" : 2,
            "ptrb" : "/main/b"
        }
    })");

    const ConfPointer ptra("/main/a");
    const ConfPointer ptrb(config.at("main").at("ptrb"));

    ASSERT_EQ(config.at(ptra).get<int>(), 1);
    ASSERT_EQ(config.at(ptrb).get<int>(), 2);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
