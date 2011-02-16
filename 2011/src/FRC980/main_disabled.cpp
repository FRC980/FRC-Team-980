#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

void Main::DisabledInit(void)
{
}

void Main::DisabledContinuous(void)
{
}

void Main::DisabledPeriodic(void)
{
    //--- Get the Robot instance
    //Robot980* pRobot = Robot980::GetInstance();

    //--- Feed the watchdog
    GetWatchdog().Feed();
}
