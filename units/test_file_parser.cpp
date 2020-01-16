#include <msode/core/file_parser.h>

#include <gtest/gtest.h>

#include <cstdio>
#include <map>
#include <string>

using namespace msode;

GTEST_TEST( FILE_PARSER, simple )
{
    const std::string filename = "tmp.dat";
    FILE *f = fopen(filename.c_str(), "w");
    fprintf(f, "key value");
    fclose(f);

    FileParser fp(filename);
    ASSERT_EQ(fp.getStr("key"), "value");
}

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
