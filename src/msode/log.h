#pragma once

#include <iostream>
#include <string>

#ifdef MSODE_FAIL_ON_CONTRACT
#define MSODE_CHECK(val, msg) do {                   \
        if (!(val)) {                           \
            std::cerr << (msg) << std::endl;    \
            exit(1);                            \
        }                                       \
    } while(0)
#else
#define MSODE_CHECK(val, msg) do {} while(0)
#endif


#define MSODE_Expect(val, msg) MSODE_CHECK(val, std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": Failed Expect : " + msg)
#define MSODE_Ensure(val, msg) MSODE_CHECK(val, std::string(__FILE__) + ":" + std::to_string(__LINE__) + ": Failed Ensure : " + msg)
