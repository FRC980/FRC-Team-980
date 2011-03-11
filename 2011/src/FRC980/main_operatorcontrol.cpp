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
    static Joystick *pjsArm   = Joystick::GetStickForPort(2);

    //--- Swap joysticks on request
    RUN_ONCE(pjsDrive, JS_IDENTIFY_ARM)
    {
        utils::message("swapping joysticks");
        std::swap(pjsDrive, pjsArm);
    }
    RUN_ONCE(pjsArm, JS_IDENTIFY_DRIVE)
    {
        utils::message("swapping joysticks");
        std::swap(pjsDrive, pjsArm);
    }
    

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
    if (pjsDrive->GetRawButton(DRIVE_SLOW_MODE) && !bSlowMode)
    {
        bSlowMode = true;
        utils::message("slow mode");
    }
    if (pjsDrive->GetRawButton(DRIVE_FAST_MODE) && bSlowMode)
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

    if(pjsDrive->GetRawButton(DRIVE_PRINT_LINETRACKER))
    {
		utils::message("Line tracker:%d", pRobot->GetLineTracker());
    }

    if(pjsDrive->GetRawButton(DRIVE_PRINT_STATUS))
    {
        pRobot->PrintState();
    }

    RUN_ONCE(pjsDrive, DRIVE_LED_NONE)
    {
        pRobot->LightLED(0);
    }

    RUN_ONCE(pjsDrive, DRIVE_LED_TRIANGLE)
    {
        pRobot->LightLED(1);
    }

    RUN_ONCE(pjsDrive, DRIVE_LED_CIRCLE)
    {
        pRobot->LightLED(2);
    }

    RUN_ONCE(pjsDrive, DRIVE_LED_SQUARE)
    {
        pRobot->LightLED(3);
    }

    //--- Arm code
    static bool target_center = false;
    static int target_position = 570;

    RUN_ONCE(pjsArm, ARM_ENABLE_TARGET_CENTER)
    {
        target_center = true;
        utils::message("Targeting center");
    }

    RUN_ONCE(pjsArm, ARM_DISABLE_TARGET_CENTER)
    {
        target_center = false;
        utils::message("Targeting center");
    }

    RUN_ONCE(pjsArm, ARM_POSITION_LOW)
    {
        target_position = target_center ? 130 : 175;
        utils::message("Position #1: %D", target_position);
    }
    RUN_ONCE(pjsArm, ARM_POSITION_MIDDLE)
    {
        target_position = target_center ? 285 : 300;
        utils::message("Position #2: %D", target_position);
    }
    RUN_ONCE(pjsArm, ARM_POSITION_HIGH)
    {
        target_position = target_center ? 430 : 465;
        utils::message("Position #3: %D", target_position);
    }

    int displacement = (int)(pjsArm->GetY() * 80);
    //pRobot->SetPosition(target_position + displacement);
    float arm_speed = -pjsArm->GetRawAxis(XB_AXIS_RIGHT_Y);
    if (arm_speed > 0.1)
    {
        //up
        pRobot->m_pscShoulder->Set((arm_speed-0.1)*(10.0/9.0));
    }
    else if (arm_speed < -0.1)
    {
        //down
        pRobot->m_pscShoulder->Set((arm_speed+0.1)/4.0);
    }
    else
    {
        pRobot->m_pscShoulder->Set(0.0);
    }

    if(pjsArm->GetRawAxis(XB_AXIS_TRIGGER) > 0.3)
    {
	    pRobot->RunClaw(1.0);
    }
    else if(pjsArm->GetRawAxis(XB_AXIS_TRIGGER) < -0.3)
    {
	    pRobot->RunClaw(-1.0);
    }
    else
    {
        pRobot->RunClaw(0.0);
    }
}
