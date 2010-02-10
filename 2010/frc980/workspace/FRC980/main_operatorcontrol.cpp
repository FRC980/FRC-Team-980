#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==============================================================================
//==============================================================================
void Main::TeleopInit()
{
    Robot980* pRobot = Robot980::GetInstance();
}

//==============================================================================
void Main::TeleopContinuous()
{
    
}

//==============================================================================
void Main::TeleopPeriodic()
{
    Robot980* pRobot = Robot980::GetInstance();

    Joystick* pjsDrive = Joystick::GetStickForPort(1);

    GetWatchdog().Feed();

    float x = pjsDrive->GetX();
    x = (x > 0) ? x * x : x * x * -1;

    float y = pjsDrive->GetY();
    y = (y > 0) ? y * y : y * y * -1;

    float fLeft  = limit(y - x);
    float fRight = limit(y + x);

    pRobot->Drive(fLeft, fRight);
}
