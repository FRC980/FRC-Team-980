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

/*
Usage:
RUN_ONCE(pjsDrive,11)
{
    //do something when button 11 is pressed
}

RUN_ONCE_VAR(pjsDrive,11,second_time)
{
    //use a different variable if the same joystick
    //button is used twice in different places
}

 */

#define RUN_ONCE_VAR(joystick,button,var)              \
    static bool var = false;                           \
    if (! joystick->GetRawButton(button))              \
    {                                                  \
         var = false;                                  \
    }                                                  \
    else if (joystick->GetRawButton(button) &&         \
             !var && (var=true))

#define RUN_ONCE(joystick,button)                      \
    RUN_ONCE_VAR(joystick,button,joystick##_##button##_pressed)


void Main::TeleopPeriodic(void)
{
    //--- Get the Robot instance
    Robot980 *pRobot = Robot980::GetInstance();

    //--- Feed the watchdog
    GetWatchdog().Feed();

    //--- Set a pointer to the joystick(s)
    static Joystick *pjsDrive = Joystick::GetStickForPort(1);

    float y = pjsDrive->GetY();
    y = (y > 0) ? y * y : y * y * -1;
    pRobot->Drive(y);

}
