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
#define MSODE_CHECK(val, pattern, ...) do {                             \
        if (!(val)) {                                                   \
            msode::die__(__FILE__, __LINE__, pattern, ##__VA_ARGS__);   \
        }                                                               \
    } while(0)
#else
#define MSODE_CHECK(val, pattern, ...) do {} while(0)
#endif


#define MSODE_Expect(val, pattern, ...) MSODE_CHECK(val, pattern, ##__VA_ARGS__)
#define MSODE_Ensure(val, pattern, ...) MSODE_CHECK(val, pattern, ##__VA_ARGS__)

#define msode_die(pattern, ...) msode::die__(__FILE__, __LINE__, pattern, ##__VA_ARGS__)
