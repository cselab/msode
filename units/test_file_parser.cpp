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
    fprintf(f, "key value\n");
    fclose(f);

    FileParser fp(filename);
    ASSERT_EQ(fp.getStr("key"), "value");
}

GTEST_TEST( FILE_PARSER, types )
{
    const std::string filename = "tmp.dat";
    FILE *f = fopen(filename.c_str(), "w");
    fprintf(f, "str hello\n");
    fprintf(f, "real3 1.1 2.2 3.3\n");
    fprintf(f, "quaternion 1.1 -2.2 3.3 -4.4\n");
    fprintf(f, "subm 1.1 -2.2 3.3\n");
    fclose(f);

    FileParser fp(filename);
    ASSERT_EQ(fp.getStr("str"), "hello");

    ASSERT_EQ(fp.getReal3("real3").x, 1.1);
    ASSERT_EQ(fp.getReal3("real3").y, 2.2);
    ASSERT_EQ(fp.getReal3("real3").z, 3.3);

    ASSERT_EQ(fp.getQuaternion("quaternion").w,  1.1);
    ASSERT_EQ(fp.getQuaternion("quaternion").x, -2.2);
    ASSERT_EQ(fp.getQuaternion("quaternion").y,  3.3);
    ASSERT_EQ(fp.getQuaternion("quaternion").z, -4.4);

    ASSERT_EQ(fp.getSubMatrix("subm")[0],  1.1);
    ASSERT_EQ(fp.getSubMatrix("subm")[1], -2.2);
    ASSERT_EQ(fp.getSubMatrix("subm")[2],  3.3);
}

static std::map<std::string, std::string> createLargeData(int nentries)
{
    std::map<std::string, std::string> data;
    
    for (int i = 0; i < nentries; ++i)
    {
        const std::string key = "key_" + std::to_string(i);
        const std::string val = "val_" + std::to_string(i*i);
        data[key] = val;
    }
    return data;
}

GTEST_TEST( FILE_PARSER, large )
{
    const auto data = createLargeData(100);
    
    const std::string filename = "tmp.dat";
    FILE *f = fopen(filename.c_str(), "w");
    for (auto d : data)
    {
        const auto key = d.first;
        const auto val = d.second;
        fprintf(f, "%s %s\n", key.c_str(), val.c_str());
    }
    fclose(f);

    FileParser fp(filename);

    for (auto d : data)
    {
        const auto key = d.first;
        const auto val = d.second;
        ASSERT_EQ(fp.getStr(d.first), d.second);
    }
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
