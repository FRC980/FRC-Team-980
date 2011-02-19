#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "jsbuttons.h"
#include "utils.h"

void Main::TeleopInit(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    pRobot->SetBrakes(false);
}

void Main::TeleopContinuous(void)
{
}

void Main::TeleopPeriodic(void)
{
    //--- Get the Robot instance
    Robot980 *pRobot = Robot980::GetInstance();

    //--- Feed the watchdog
    GetWatchdog().Feed();

    //--- Set a pointer to the joystick(s)
    Joystick *pjsDrive = Joystick::GetStickForPort(1);
    Joystick *pjsArm   = Joystick::GetStickForPort(2);

    //--- Get the x and y position from the joystick
    float x = pjsDrive->GetX();
    x = (x > 0) ? x * x : x * x * -1;

    // pushing joystick away is negative Y, but we want it to be
    // "forward", which should be positive, so we have a "-" here
    float y = -pjsDrive->GetY();
    y = (y > 0) ? y * y : y * y * -1;

    //--- Get the speed for the left and right motors
    //    NOTE: The speed is limited in the Drive() method for safety
    float fLeft = (y + x);
    float fRight = (y - x);



    // Slow mode imposes a speed limit
    // Left thumb button on joystick enables, right thumb button disables
    static bool bSlowMode = false;
    if (pjsDrive->GetRawButton(JS_TOP_LEFT) && !bSlowMode)
    {
        bSlowMode = true;
        utils::message("slow mode");
    }
    if (pjsDrive->GetRawButton(JS_TOP_RIGHT) && bSlowMode)
    {
        bSlowMode = false;
        utils::message("fast mode");
    }

    if (bSlowMode)
    {
        pRobot->Drive(fLeft/2.0, fRight/2.0);
    }
    else
    {
        pRobot->Drive(fLeft, fRight);
    }
}
