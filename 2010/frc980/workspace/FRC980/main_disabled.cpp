#include <WPILib.h>

#include "main.h"

#include "Robot980.h"

// This function gets called once, each time the robot enters disabled
// mode.
void Main::DisabledInit()
{}

// This function gets called continuously while in disabled mode.
void Main::DisabledContinuous()
{}

// This function gets called each time new data is received from the
// driver station, while in disabled mode.
void Main::DisabledPeriodic()
{
    GetWatchdog().Feed();
}
