#include <DriverStation.h>

#include "utils.h"

double limit(double val, double min /* = -1 */, double max /* = 1 */)
{
    if (val > max)
        return max;

    if (val < min)
        return min;

    return val;
}

const char* const AllianceChar(enum DriverStation::Alliance a)
{
    if (DriverStation::kRed == a)
        return "R";
    if (DriverStation::kBlue == a)
        return "B";
    if (DriverStation::kInvalid == a)
        return "I";
    return "X";
}
