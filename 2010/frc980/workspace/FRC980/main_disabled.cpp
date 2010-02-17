#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==============================================================================
//==============================================================================
void Main::DisabledInit()
{
    
}

//==============================================================================
void Main::DisabledContinuous()
{
    
}

//==============================================================================
void Main::DisabledPeriodic()
{
    GetWatchdog().Feed();
}
