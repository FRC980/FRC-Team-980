#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==========================================================================
static int iMode = 0;
static Timer *pTimerAuton = new Timer;

//==========================================================================
void Main::AutonomousInit(void)
{
    Robot980* pRobot = Robot980::GetInstance();
    iMode = pRobot->GetAutonMode();
    iMode = 6;

    pRobot->SetBrakes(true);

    pTimerAuton->Start();
    pTimerAuton->Reset();

    utils::message("Running autonomous mode %d\n", iMode);
}

//==========================================================================
void Main::AutonomousContinuous(void)
{
}

//==========================================================================
void Main::AutonomousPeriodic(void)
{
    //--- Feed the watchdog
    GetWatchdog().Feed();

    //--- Switch to the correct autonomous mode
    switch(iMode)
    {
    default:

    case 1:
        this->Auton1();
        break;

    case 2:
        this->Auton2();
        break;

    case 3:
        this->Auton3();
        break;

    case 4:
        this->Auton4();
        break;

    case 5:
        this->Auton5();
        break;

    case 6:
        this->Auton6();
        break;
    }
}

//==========================================================================
void Main::Auton1(void)
{
    // drive 8 feet, kick, then turn right 90 degrees
    Robot980* pRobot = Robot980::GetInstance();

    static enum {
        START,
        DRIVE_8_ft,
        KICK_1,
        TURN_right,
    } auton1State = START;

    static Timer *pTimer = new Timer;

    switch (auton1State)
    {
    case START:
    {
        if (!pRobot->KickerReady())
            pRobot->ArmKicker();
        pTimer->Start();
        pTimer->Reset();
        auton1State = DRIVE_8_ft;
    }
    // fall through

    case DRIVE_8_ft:
    {
        // TODO: user encoders to measure distance
    }
    break;

    case KICK_1:
    {
    }
    break;

    case TURN_right:
    {
    }
    break;
    }
}

//==========================================================================
void Main::Auton2(void)
{
    // drive 8 feet, kick, drive 3 feet, kick, turn left 90 degrees
}

//==========================================================================
void Main::Auton3(void)
{
    // drive 8 feet, kick, drive 3 feet, kick, drive 3 feet, kick, turn right
}

//==========================================================================
void Main::Auton4(void)
{
    // drive 18 feet, then turn right 90 degrees
}

//==========================================================================
void Main::Auton5(void)
{

}

//==========================================================================
void Main::Auton6(void)
{
    //--- Get the Robot instance
    Robot980* pRobot = Robot980::GetInstance();

    //--- Get the autonomous mode timer in seconds
    float t = pTimerAuton->Get();

    //--- In the first few seconds of the match drive forward
    if (t < 1.5)
    {
        pRobot->Drive(0.5, 0.5, 0.2); // left, right, roller
    }

    //--- After two seconds stop the robot and fire
    if (t >= 2.0)
    {
        pRobot->Drive(0, 0, 0); // stop

        static bool bFired = true;

        if (! bFired)
        {
            pRobot->FireKicker();
            bFired = true;
        }
    }

    //--- After three and a half seconds rearm the kicker
    if (t > 3.5)
    {
        pRobot->ArmKicker();
    }
}
