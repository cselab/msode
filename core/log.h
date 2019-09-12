#pragma once

#include <iostream>
#include <string>

#ifdef FAIL_ON_CONTRACT
#define _CHECK(val, msg) do {                   \
        if (!(val)) {                           \
            std::cerr << (msg) << std::endl;    \
            exit(1);                            \
        }                                       \
    } while(0)
#else
#define _CHECK(val, msg) do {} while(0)
#endif


#define Expect(val, msg) _CHECK(val, std::string("Failed Expect : ") + msg)
#define Ensure(val, msg) _CHECK(val, std::string("Failed Ensure : ") + msg)
