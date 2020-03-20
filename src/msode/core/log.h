#pragma once

#include <cstdio>
#include <iostream>
#include <string>


namespace msode
{
void die__ [[gnu::format(printf, 3, 4)]] 
(const char *file, int line, const char* pattern, ...);
void printStacktrace();
} // namespace msode

#ifdef MSODE_FAIL_ON_CONTRACT
#define MSODE_CHECK(val, msg) do {                   \
        if (!(val)) {                           \
            std::cerr << (msg) << std::endl;    \
            printStacktrace();                  \
            exit(1);                            \
        }                                       \
    } while(0)
#else
#define MSODE_CHECK(val, msg) do {} while(0)
#endif


#define MSODE_Expect(val, msg) MSODE_CHECK(val, std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": Failed Expect : " + msg)
#define MSODE_Ensure(val, msg) MSODE_CHECK(val, std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": Failed Ensure : " + msg)

#define msode_die(pattern, ...) msode::die__(__FILE__, __LINE__, pattern, ##__VA_ARGS__)
