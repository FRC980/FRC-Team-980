#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==============================================================================
//==============================================================================
void Main::TeleopInit()
{
   Robot980::GetInstance();
}

//==============================================================================
void Main::TeleopContinuous()
{
   
}

//==============================================================================
void Main::TeleopPeriodic()
{
   //--- Get the Robot instance
   Robot980* pRobot = Robot980::GetInstance();

   //--- Set a pointer to the joystick
   Joystick* pjsDrive = Joystick::GetStickForPort(1);

   //--- Feed the watchdog after getting data from Joystick in case Joystick hangs
   GetWatchdog().Feed();
   
   //--- Get the x and y position from the joystick
   float x = pjsDrive->GetX();
   x = (x > 0) ? x * x : x * x * -1;

   float y = pjsDrive->GetY();
   y = (y > 0) ? y * y : y * y * -1;
   
   //--- Get the speed for the left and right motors
   //    NOTE: The speed is limited in the Drive() method for safety
   float fLeft  = (y - x);
   float fRight = (y + x);

   //--- Drive the robot
   pRobot->Drive(fLeft, fRight);
}
