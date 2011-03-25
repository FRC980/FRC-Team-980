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

    if(pjsDrive->GetRawButton(DRIVE_LED_TRIANGLE))
    {
        pRobot->LightLED(LED_TRIANGLE);
    }
    
    if(pjsDrive->GetRawButton(DRIVE_LED_CIRCLE))
    {
        pRobot->LightLED(LED_CIRCLE);
    }
    
    if(pjsDrive->GetRawButton(DRIVE_LED_SQUARE))
    {
        pRobot->LightLED(LED_SQUARE);
    }

    if(pjsDrive->GetRawButton(DRIVE_LED_NONE))
    {
        pRobot->LightLED(LED_OFF);
    }

    //--- Minibot deployment
    if(    (pjsArm->GetRawButton(XB_BUTTON_BACK)  )
        && (pjsArm->GetRawButton(XB_BUTTON_START) ))
    {
        // Right joystick up and right trigger pressed
        pRobot->Deploy(-0.7);
        utils::message("deploying minibot");
    }
    else
    {
        pRobot->Deploy(0.0);
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
        target_position = POT_GROUND;
        utils::message("Ground position: %D", target_position);
    }
    else if(pjsArm->GetRawButton(ARM_POSITION_LOW))
    {
        target_position = target_center ? POT_CENTER_LOW : POT_SIDE_LOW;
        utils::message("Position #1: %D", target_position);
    }
    else if(pjsArm->GetRawButton(ARM_POSITION_MIDDLE))
    {
        target_position = target_center ? POT_CENTER_MIDDLE : POT_SIDE_MIDDLE;
        utils::message("Position #2: %D", target_position);
    }
    else if(pjsArm->GetRawButton(ARM_POSITION_HIGH))
    {
        target_position = target_center ? POT_CENTER_HIGH : POT_SIDE_HIGH;
        utils::message("Position #3: %D", target_position);
    }
    else if(pjsArm->GetRawButton(ARM_POSITION_MOVING))
    {
        target_position = POT_CARRY;
        utils::message("Position for moving around: %D", target_position);
    }
    else
    {
        target_position = -1;
    }

    if (target_position != -1)
    {
        int displacement = (int)(-pjsArm->GetY() * 110);
        pRobot->SetPosition(target_position + displacement);
    }
    else
    {
        float arm_js_speed = -pjsArm->GetY();
        pRobot->SetArmSpeed(arm_js_speed);
    }

    static bool close_pressed = false;
    static bool open_pressed = false;

    if( !open_pressed
       && (pjsArm->GetRawAxis(XB_AXIS_TRIGGER) > 0.3)  )
    {
        pRobot->OpenClaw();

        utils::message("Opening claw");
        open_pressed=true;
        close_pressed=false;
    }
    else if( !close_pressed
            && (pjsArm->GetRawAxis(XB_AXIS_TRIGGER) < -0.3)  )
    {
        pRobot->CloseClaw();

        utils::message("Closing claw");
        open_pressed=false;
        close_pressed=true;
    }
    else
    {
        open_pressed=false;
        close_pressed=false;
    }

    if (pRobot->GetClawTimer() > 0.3 && pRobot->GetClawCurrent() > 30.0)
    {
        utils::message("Claw disabled at current=%f", pRobot->GetClawCurrent());
        pRobot->RunClaw(0.0);
    }

    if (pRobot->GetClawTimer() > 1.5 )
    {
        pRobot->RunClaw(0.0);
    }
    utils::message("Encoders L:%f R:%f",pRobot->GetLeftEncoder()/21.8,pRobot->GetRightEncoder()/21.8);
}
