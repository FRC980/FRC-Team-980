#ifndef UTILS_H
#define UTILS_H

#include <DriverStation.h>

double limit(double val, double min = -1, double max = 1);
const char* const AllianceChar(enum DriverStation::Alliance a);

#define ABS(x)	(((x) > 0) ? (x) : (-x))

#endif // UTILS_H
