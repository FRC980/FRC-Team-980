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
    Robot980* pRobot = Robot980::GetInstance();
    pRobot->SetBrakes(false);
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

    //--- Get the x and y position from the joystick
    float x = pjsDrive->GetX();
    if (bSlowMode) x = x / 2.0;
    x = (x > 0) ? x * x : x * x * -1;

    // pushing joystick away is negative Y, but we want it to be
    // "forward", which should be positive, so we have a "-" here
    float y = - pjsDrive->GetY();
    if (bSlowMode) y = y / 2.0;
    y = (y > 0) ? y * y : y * y * -1;

    //--- Get the speed for the left and right motors
    //    NOTE: The speed is limited in the Drive() method for safety
    float fLeft  = (y + x);
    float fRight = (y - x);


    static Timer* pTimerRoller = NULL;
    if (! pTimerRoller)
    {
        pTimerRoller = new Timer;
        pTimerRoller->Reset();
        pTimerRoller->Start();
    }

    //--- Drive the robot
    if (pRobot->BallPresent())
        pTimerRoller->Reset();
    // roller runs "slow" until sensor detects a ball, then it runs "fast"
    // until a few seconds after loosing the ball.
    pRobot->Drive(fLeft, fRight, (pTimerRoller->Get() < 3.0)
                  ? pjsKick->GetZ() : (pjsKick->GetZ() / 3));

    //Debug lift switches
    if (pjsDrive->GetRawButton(11))
    {
        pRobot->PrintLiftSwitchStatus();
    }

    //--- Debug: run winch from joystick
    static bool bRunWinchJs = false;
    if (pjsKick->GetRawButton(4))
    {
        pRobot->RunWinch(pjsKick->GetY());
        bRunWinchJs = true;
    }
    else if (bRunWinchJs)
    {
        pRobot->RunWinch(0);
        bRunWinchJs = false;
    }
    else
    {
        pRobot->Lift(- pjsKick->GetY());
    }

    if (pjsKick->GetRawButton(JS_TRIGGER))
    {
        if (pRobot->FireKicker())
        {
            utils::message("FIRING\n");
        }
        else
        {
            utils::message("not firing\n");
        }
    }

    if (pjsKick->GetRawButton(6))
    {
        pRobot->SetAutoUnwind(true);
    }

    if (pjsKick->GetRawButton(7))
    {
        pRobot->SetAutoUnwind(false);
    }


    if (pjsKick->GetRawButton(JS_TOP_BOTTOM))
    {
        pRobot->ArmKicker();
    }

    if (pjsKick->GetRawButton(JS_TOP_CENTER))
    {
        pRobot->Unwind();
    }

    if (pjsKick->GetRawButton(JS_RIGHT_TOP))
    {
        pRobot->PrintState();
    }

    /*
    if (pjsKick->GetRawButton())
    {
        pRobot->ArmingEnable();
    }

    if (pjsKick->GetRawButton())
    {
        pRobot->ArmingDisable();
    }*/
    
    //--- DEBUG: set states manually
    if (pjsKick->GetRawButton(8))
    {
        pRobot->SetState(Robot980::WOUND);
    }
    
    if (pjsKick->GetRawButton(9))
    {
        pRobot->SetState(Robot980::UNKNOWN);
    }
}
