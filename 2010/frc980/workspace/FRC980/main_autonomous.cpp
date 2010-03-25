#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==========================================================================
static int iMode = 0;
static Timer *pTimerAuton = new Timer;

void Auton2404(void);

void Auton1(void);
void Auton2(void);
void Auton3(void);
void Auton4(void);
void Auton5(void);
void Auton6(void);


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
    case 2:
    case 3:
        Auton2404(iMode);
        break;

    case 4:
        Auton4();
        break;

    case 5:
        Auton5();
        break;

    case 6:
        Auton6();
        break;
    }
}

#define DRIVE_TIME_18FT     5.5 /* seconds */
#define DRIVE_SPEED_18FT    0.5

#define DRIVE_TIME_8FT      3.0 /* seconds */
#define DRIVE_SPEED_8FT     0.5

#define DRIVE_TIME_3FT      1.5 /* seconds */
#define DRIVE_SPEED_3FT     0.5

#define TURN_TIME_90_DEG    1.0 /* seconds */
#define TURN_SPEED_90_DEG   0.5

//==========================================================================
void Auton2404(int iMode)
{
    // 1: drive 8 feet, kick, then turn right 90 degrees
    // 2: drive 8 feet, kick, drive 3 feet, kick, turn left 90 degrees
    // 3: drive 8 feet, kick, drive 3 feet, kick, drive 3 feet, kick, turn right

    Robot980* pRobot = Robot980::GetInstance();

    static enum {
        START,
        DRIVE_8_FT,
        KICK_1,
        DRIVE_3_FT_1,
        KICK_2,
        DRIVE_3_FT_2,
        KICK_3,
        TURN_RIGHT,
        TURN_LEFT,
        STOP,
    } autonState = START;

    static Timer *pTimer = new Timer;

    switch (autonState)
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
        pRobot->Drive(DRIVE_SPEED_8FT, DRIVE_SPEED_8FT);
        if (pTimer->Get() >= DRIVE_TIME_8FT)
            autonState++;
    }
    break;

    case KICK_1:
    {
        pRobot->Drive(0,0);
        if (pRobot->FireKicker())
        {
            pTimer->Start();
            pTimer->Reset();

            if (1 == iMode)
                autonState = TURN_RIGHT;
            else
                autonState++;
        }
    }
    break;

    case KICK_2:
    {
        pRobot->Drive(0,0);
        if (pRobot->FireKicker())
        {
            pTimer->Start();
            pTimer->Reset();

            if (2 == iMode)
                autonState = TURN_LEFT;
            else
                autonState++;
        }
    }
    break;

    case KICK_3:
    {
        pRobot->Drive(0,0);
        if (pRobot->FireKicker())
        {
            pTimer->Start();
            pTimer->Reset();

            autonState++;
        }
    }
    break;

    case DRIVE_3_FT_1:
    case DRIVE_3_FT_2:
    {
        // TODO: user encoders to measure distance
        pRobot->Drive(DRIVE_SPEED_3FT, DRIVE_SPEED_3FT);
        if (pTimer->Get() >= DRIVE_TIME_3FT)
        {
            autonState++;
        }
    }
    break;

    case TURN_RIGHT:
    {
        // TODO: user encoders to measure distance
        pRobot->Drive(TURN_SPEED_90_DEG, - TURN_SPEED_90_DEG);
        if (pTimer->Get() >= TURN_TIME_90_DEG)
        {
            autonState = STOP;
        }
    }
    break;

    case TURN_LEFT:
    {
        // TODO: user encoders to measure distance
        pRobot->Drive(- TURN_SPEED_90_DEG, TURN_SPEED_90_DEG);
        if (pTimer->Get() >= TURN_TIME_90_DEG)
        {
            autonState = STOP;
        }
    }
    break;

    case STOP:
    {
        pRobot->Drive(0, 0, 0);
    }
    break;
    }
}

//==========================================================================
void Auton1(void)
{
    // drive 8 feet, kick, then turn right 90 degrees
}

//==========================================================================
void Auton2(void)
{
    // drive 8 feet, kick, drive 3 feet, kick, turn left 90 degrees
}

//==========================================================================
void Auton3(void)
{
    // drive 8 feet, kick, drive 3 feet, kick, drive 3 feet, kick, turn right
}

//==========================================================================
void Auton4(void)
{
    // drive 18 feet, then turn right 90 degrees
    // TODO: Current version is drive 18 feet, then stop w/o turning

    //--- Get the Robot instance
    Robot980* pRobot = Robot980::GetInstance();

    //--- Get the autonomous mode timer in seconds
    float t = pTimerAuton->Get();

    //--- In the first few seconds of the match drive forward
    if (t < DRIVE_TIME_18FT)
    {
        pRobot->Drive(DRIVE_SPEED_18FT, DRIVE_SPEED_18FT);
    }
    else
    {
        pRobot->Drive(0, 0, 0); // TODO: Turn right 90 degrees
    }
}

//==========================================================================
void Auton5(void)
{

}

//==========================================================================
void Auton6(void)
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
