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

#define DEBUG_JOYSTICK
#ifdef DEBUG_JOYSTICK
    static Joystick *pjsDebug   = Joystick::GetStickForPort(3);
#endif

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
	if (pjsDrive->GetRawButton(DRIVE_ALIGN_MINIBOT))
	{
        pRobot->Align(-1.0);
        utils::message("aligning minibot");
	}
    else
    {
        pRobot->Align(0.0);
    }

	if (pjsArm->GetRawButton(ARM_DEPLOY))
	{
        pRobot->Deploy(-1.0);
        utils::message("deploying minibot");
	}
    else
    {
        pRobot->Deploy(0.0);
    }


    //--- Arm code
    static int target_position = -1;

	if(pjsArm->GetRawButton(ARM_POSITION_GROUND))
	{
        target_position = POT_GROUND;
        utils::message("Ground position: %D", target_position);
	}

	else if(pjsArm->GetRawButton(ARM_POSITION_SIDE_LOW))
	{
        target_position = POT_SIDE_LOW;
        utils::message("Side low position: %D", target_position);
	}
	else if(pjsArm->GetRawButton(ARM_POSITION_SIDE_MIDDLE))
	{
        target_position = POT_SIDE_MIDDLE;
        utils::message("Side middle position: %D", target_position);
	}
	else if(pjsArm->GetRawButton(ARM_POSITION_SIDE_HIGH))
	{
        target_position = POT_SIDE_HIGH;
        utils::message("Side high position: %D", target_position);
	}

	else if(pjsArm->GetRawButton(ARM_POSITION_CENTER_LOW))
	{
        target_position = POT_CENTER_LOW;
        utils::message("Center low position: %D", target_position);
	}
	else if(pjsArm->GetRawButton(ARM_POSITION_CENTER_MIDDLE))
	{
        target_position = POT_CENTER_MIDDLE;
        utils::message("Center middle position: %D", target_position);
	}
	else if(pjsArm->GetRawButton(ARM_POSITION_CENTER_HIGH))
	{
        target_position = POT_CENTER_HIGH;
        utils::message("Center high position: %D", target_position);
	}

    else if(pjsArm->GetRawButton(ARM_POSITION_CARRY))
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
        int displacement = (int)(pjsArm->GetX() * 110);
        pRobot->SetPosition(target_position + displacement);
    }
    else
    {
#ifdef DEBUG_JOYSTICK
        float arm_js_speed = -pjsDebug->GetY();
        pRobot->SetArmSpeed(arm_js_speed);
#else
        pRobot->SetArmSpeed(0);
#endif
    }



	static bool claw_open = false;

	if(pjsArm->GetRawButton(ARM_CLAW) && !claw_open)
	{
		// If claw is closed and switch is flipped
		pRobot->OpenClaw();
		claw_open = true;
	}
	else if(!pjsArm->GetRawButton(ARM_CLAW) && claw_open)
	{
		// If claw is open and switch is flipped
		pRobot->CloseClaw();
		claw_open = false;
	}

    // Stop the claw, if necessary
    pRobot->CheckStopClaw();


#ifdef DEBUG_JOYSTICK
    //--- Debug joystick code
    if(pjsDebug->GetRawButton(DEBUG_PRINT_LINETRACKER))
    {
		utils::message("Line tracker:%d", pRobot->GetLineTracker());
    }

    if(pjsDebug->GetRawButton(DEBUG_PRINT_ARM_STATE))
    {
		pRobot->PrintState();
    }

    if(pjsDebug->GetRawButton(DEBUG_AUTON_INIT))
    {
		utils::message("Manually initializing autonomous");
		AutonomousInit();
    }

    if(pjsDebug->GetRawButton(DEBUG_RUN_AUTON_MODE))
    {
		AutonomousPeriodic();
    }

#endif
}
