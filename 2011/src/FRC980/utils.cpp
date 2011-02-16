#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <NetworkCommunication/FRCComm.h>

#include "utils.h"

utils::utils(void)
{
}

utils::~utils(void)
{
}

double utils::limit(double val, double min /* = -1 */ ,
                    double max /* = 1 */ )
{
    if (val > max)
        return max;

    if (val < min)
        return min;

    return val;
}

void utils::message(char *fmt, ...)
{
    char message[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(message, 256, fmt, args);
    va_end(args);

    setErrorData(message, strlen(message), 100);
}
