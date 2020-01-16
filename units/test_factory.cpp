#include <msode/core/factory.h>

#include <gtest/gtest.h>

#include <cstdio>
#include <string>

using namespace msode;

GTEST_TEST( FACTORY, rigidBody )
{
    const std::string filename = "tmp.dat";
    FILE *f = fopen(filename.c_str(), "w");
    fprintf(f,
            "m 0.0 10.000000 0.0\n"
            "q 0.0 1.0 0.0 0.0\n"
            "r 0.0 0.0 0.0\n"
            "A 0.246391 0.200475 0.199631\n"
            "B 0.100000 0.000000 0.000000\n"
            "C 6.983636 1.153988 1.218122\n");
    fclose(f);

    const auto body = Factory::readRigidBodyConfig(filename);

    ASSERT_EQ(body.magnMoment.x, 0.0_r);
    ASSERT_EQ(body.magnMoment.y, 10.0_r);
    ASSERT_EQ(body.magnMoment.z, 0.0_r);

    ASSERT_EQ(body.propulsion.A[0], 0.246391_r);
    ASSERT_EQ(body.propulsion.A[1], 0.200475_r);
    ASSERT_EQ(body.propulsion.A[2], 0.199631_r);

    ASSERT_EQ(body.propulsion.B[0], 0.1_r);
    ASSERT_EQ(body.propulsion.B[1], 0.0_r);
    ASSERT_EQ(body.propulsion.B[2], 0.0_r);

    ASSERT_EQ(body.propulsion.C[0], 6.983636_r);
    ASSERT_EQ(body.propulsion.C[1], 1.153988_r);
    ASSERT_EQ(body.propulsion.C[2], 1.218122_r);
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}