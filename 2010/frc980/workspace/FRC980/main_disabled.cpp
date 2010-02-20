#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==============================================================================
//==============================================================================
void Main::DisabledInit(void)
{
    
}

//==============================================================================
void Main::DisabledContinuous(void)
{
    
}

//==============================================================================
void Main::DisabledPeriodic(void)
{
    GetWatchdog().Feed();
}
