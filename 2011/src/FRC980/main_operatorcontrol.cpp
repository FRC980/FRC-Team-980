#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "jsbuttons.h"
#include "utils.h"

static Timer *pTimerClaw = new Timer;

void Main::TeleopInit(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    pRobot->SetBrakes(false);
    pTimerClaw->Start();
    pTimerClaw->Reset();
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

    //--- LED signal lights
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

    //--- Minibot deployment
    if(pjsDrive->GetRawAxis(XB_AXIS_TRIGGER) < -0.3)
    {
        // D-pad - safety for minibot deployment
        if (pjsDrive->GetRawAxis(XB_AXIS_DPAD_Y) == -1)
        {
            pRobot->Deploy(0.2);
            utils::message("deploying minibot");
        }
        else
        {
            utils::message("safety is off");
            pRobot->Deploy(0.0);
        }
    }

    //--- Additional code on drive joystick
    if(pjsDrive->GetRawButton(DRIVE_PRINT_LINETRACKER))
    {
		utils::message("Line tracker:%d", pRobot->GetLineTracker());
    }




    //--- Arm code
    if(pjsArm->GetRawButton(ARM_PRINT_STATUS))
    {
        pRobot->PrintState();
    }

    static bool target_center = false;
    static int target_position = -1;

    RUN_ONCE(pjsArm, ARM_ENABLE_TARGET_CENTER)
    {
        target_center = true;
        utils::message("Targeting center");
    }

    RUN_ONCE(pjsArm, ARM_DISABLE_TARGET_CENTER)
    {
        target_center = false;
        utils::message("Targeting side");
    }
    if(pjsArm->GetRawButton(ARM_POSITION_GROUND))
    {
        target_position = 30;
        utils::message("Position #3: %D", target_position);
    }
    else if(pjsArm->GetRawButton(ARM_POSITION_LOW))
    {
        target_position = target_center ? 175 : 130;
        utils::message("Position #1: %D", target_position);
    }
    else if(pjsArm->GetRawButton(ARM_POSITION_MIDDLE))
    {
        target_position = target_center ? 300 : 285;
        utils::message("Position #2: %D", target_position);
    }
    else if(pjsArm->GetRawButton(ARM_POSITION_HIGH))
    {
        target_position = target_center ? 465 : 430;
        utils::message("Position #3: %D", target_position);
    }
    else
    {
        target_position = -1;
    }

    float arm_js_speed = -pjsArm->GetY();

    if (arm_js_speed > 0.1)
    {
        target_position = -1;
        pRobot->SetArmSpeed(arm_js_speed - 0.1);
    }
    else if (arm_js_speed < -0.1)
    {
        target_position = -1;
        pRobot->SetArmSpeed((arm_js_speed + 0.1)/4.0);
    }

    if (target_position != -1)
    {
        int displacement = (int)(-pjsArm->GetRawAxis(XB_AXIS_RIGHT_Y) * 110);
        pRobot->SetPosition(target_position + displacement);
    }

    if(pjsArm->GetRawAxis(XB_AXIS_TRIGGER) > 0.3)
    {
        pRobot->RunClaw(0.9);
        pTimerClaw->Reset();
    }
    else if(pjsArm->GetRawAxis(XB_AXIS_TRIGGER) < -0.3)
    {
        pRobot->RunClaw(-0.9);
        pTimerClaw->Reset();
    }
    if (pTimerClaw->Get() > 1.0)
    {
        pRobot->RunClaw(0.0);
    }
}
