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

   //--- Set a pointer to the joystick
   Joystick* pjsDrive = Joystick::GetStickForPort(1);

   //--- Feed the watchdog
   GetWatchdog().Feed();
   
   //--- Get the x and y position from the joystick
   float x = pjsDrive->GetX();
   x = (x > 0) ? x * x : x * x * -1;

   float y = pjsDrive->GetY();
   y = (y > 0) ? y * y : y * y * -1;
   
   //--- Limit the final speed
   utils u;
   float fLeft  = u.limit(y - x);
   float fRight = u.limit(y + x);

   //--- Drive the robot
   pRobot->Drive(fLeft, fRight);
}
