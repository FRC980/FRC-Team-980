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

   //setUserDsLcdData("main teleop periodic",20,100);

   //--- Feed the watchdog
   GetWatchdog().Feed();

   //--- Set a pointer to the joystick(s)
   Joystick* pjsDrive = Joystick::GetStickForPort(1);
   Joystick* pjsKick  = Joystick::GetStickForPort(2);

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
   pRobot->Drive(fLeft, fRight, pjsKick->GetZ());

   if (pjsKick->GetRawButton(JOYSTICK_TRIGGER))
   {
       pRobot->FireKicker();
   }

   if (pjsKick->GetRawButton(JOYSTICK_THUMB_BOTTOM))
   {
       pRobot->ArmKicker();
   }

   //--- Top joystick button Lifts the Robot
   //if(pjsKick->GetRawButton(JOYSTICK_THUMB_TOP)){
   //   pRobot->Lift();
   //}

   if (pjsKick->GetRawButton(7))
       pRobot->ArmingEnable();

   if (pjsKick->GetRawButton(6))
       pRobot->ArmingDisable();

   pRobot->HandleAutomatic();   // REVIEW: This will be moved to a timer

   if (pjsDrive->GetRawButton(7))
       pRobot->SetWinch(pjsDrive->GetZ());
}
