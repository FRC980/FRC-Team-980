#include "utils.h"

double limit(double val, double min /* = -1 */, double max /* = 1 */)
{
    if (val > max)
        return max;

    if (val < min)
        return min;

    return val;
}
