#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

static int iMode = 0;

// This function gets called once, each time the robot enters autonomous
// mode.
void Main::AutonomousInit()
{
    Robot980* pRobot = Robot980::GetInstance();
    iMode = pRobot->GetAutonMode();
}

// This function gets called continuously while in autonomous mode.
void Main::AutonomousContinuous()
{}

// This function gets called each time new data is received from the
// driver station, while in autonomous mode.
void Main::AutonomousPeriodic()
{
    Robot980* pRobot = Robot980::GetInstance();
    GetWatchdog().Feed();
}
