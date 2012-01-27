#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "jsbuttons.h"
#include "utils.h"
#include "CANJaguar.h"

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
    static CANJaguar *jag1 = new CANJaguar(1);
    static bool init = false;
    if(init == false)
    {
    	jag1->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
    	jag1->SetPID(10.0, 0.0, 0.0);
    	jag1->EnableControl();
	init = true;
    }
    float y = pjsDrive->GetY();
    float x = pjsDrive->GetX();
    printf("\nx: %f", x);
    printf("\ny: %f", y);
    if(pjsDrive->GetRawButton(1))
    {
	printf("\nbutton");
    }
}
