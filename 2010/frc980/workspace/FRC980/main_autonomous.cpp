#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==============================================================================
//==============================================================================
static int iMode = 0;


//==============================================================================
//==============================================================================
void Main::AutonomousInit()
{
    Robot980* pRobot = Robot980::GetInstance();
    iMode = pRobot->GetAutonMode();
}

//==============================================================================
void Main::AutonomousContinuous()
{
    
}

//==============================================================================
void Main::AutonomousPeriodic()
{
    Robot980* pRobot = Robot980::GetInstance();
    GetWatchdog().Feed();
}
