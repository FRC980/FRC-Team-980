#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==============================================================================
//==============================================================================
void Main::TeleopInit(void)
{
   Robot980::GetInstance();
}

//==============================================================================
void Main::TeleopContinuous(void)
{
   
}

//==============================================================================
void Main::TeleopPeriodic(void)
{
   //--- Get the Robot instance
   Robot980* pRobot = Robot980::GetInstance();

   //--- Feed the watchdog
   GetWatchdog().Feed();

   //--- Set a pointer to the joystick(s)
   Joystick* pjsDrive = Joystick::GetStickForPort(1);
   
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
   
   //--- Fire and Re-Arm the Kicker
   if(pjsDrive->GetRawButton(JOYSTICK_TRIGGER)){
	   //--- Fire the Kicker
	   //pRobot->FireKicker();
   }
   
   //--- Stop the fire Cam if it's still moving
   //pRobot->StopKickerCam();
   
   //--- Re-arm the Kicker
   //pRobot->ArmKicker();
   
   //--- Stop the arming winch and unload it
   //pRobot->StopArmWinch();
   
   //--- Unwind the winch
   //pRobot->UnwindWinch();
   
   //--- Lift the Robot
   //if(pjsDrive->GetRawButton(JOYSTICK_THUMB_TOP)){
   //	   pRobot->Lift();
   //}
}
