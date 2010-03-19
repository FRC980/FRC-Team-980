#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "jsbuttons.h"
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

   //--- Set Driver Station LCD Data Display
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

   if (pjsKick->GetRawButton(JS_TRIGGER))
   {
       pRobot->FireKicker();
   }

   if (pjsKick->GetRawButton(JS_TOP_BOTTOM))
   {
       pRobot->ArmKicker();
   }

   //--- Top joystick button Lifts the Robot
   //if(pjsKick->GetRawButton(JS_TOP_CENTER)){
   //   pRobot->Lift();
   //}

   if (pjsKick->GetRawButton(JS_LEFT_BOTTOM)){
       pRobot->ArmingEnable();
   }

   if (pjsKick->GetRawButton(JS_LEFT_TOP))
   {
       pRobot->ArmingDisable();
       pRobot->SetWinch(0);
   }

   {
       // On pressing buttons 10 or 11, disable the arming, and run the
       // winch at a constant speed.  On release, stop the winch but don't
       // re-enable.
       static bool bPressed_last = false;
       bool bPressed = (pjsKick->GetRawButton(JS_RIGHT_TOP) ||
                        pjsKick->GetRawButton(JS_RIGHT_BOTTOM));

       if (bPressed){
           pRobot->ArmingDisable();
       }

       if (pjsKick->GetRawButton(JS_RIGHT_TOP)){
           pRobot->SetWinch(0.25); // wind
       }

       if (pjsKick->GetRawButton(JS_RIGHT_BOTTOM)){
           pRobot->SetWinch(0.25 * REVERSE_DRIVE); // unwind
       }

       if (!bPressed && bPressed_last){ // on release...
           pRobot->SetWinch(0);         // stop
       }

       bPressed_last = bPressed;
   }
}
