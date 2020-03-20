#include "log.h"

#define BACKWARD_HAS_BFD 1
#include "../../../extern/backward-cpp/backward.hpp"

#include <iostream>
#include <stdarg.h>

namespace msode
{

backward::SignalHandling sh;

static void getStacktrace(std::ostream& stream, size_t traceCntMax = 100)
{
    using namespace backward;

    StackTrace st;
    st.load_here(traceCntMax);
    Printer p;
    p.object = true;
    p.color_mode = ColorMode::automatic;
    p.address = true;
    p.print(st, stream);
}

void printStacktrace()
{
    getStacktrace(std::cerr);
}

void die__(const char *file, int line, const char* pattern, ...)
{
    fprintf(stderr, "[msode] an error occured on %s:%d:\n", file, line);

    va_list args;
    va_start(args, pattern);
    vfprintf(stderr, pattern, args);
    va_end(args);

    fprintf(stderr, "\n\n");

    printStacktrace();

    exit(1);
}

} // namespace msode
