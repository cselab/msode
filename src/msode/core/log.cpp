#include "log.h"

#include <stdarg.h>

namespace msode
{

void die(const char *file, int line, const char* pattern, ...)
{
    fprintf(stderr, "[msode] an error occured on %s:%d:\n", file, line);

    va_list args;
    va_start(args, pattern);
    vfprintf(stderr, pattern, args);
    va_end(args);
    
    exit(1);
}

} // namespace msode
