#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==========================================================================
static int iMode = 0;
static Timer *pTimerAuton = new Timer;

void Auton2404(int iMode);

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
    iMode = 5;

    pRobot->SetBrakes(false);

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

    default:
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

    typedef enum {
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
    } autonState_t;
    static autonState_t autonState = START;

    static Timer *pTimer = new Timer;

    switch (autonState)
    {
    case START:
    {
        if (!pRobot->KickerReady())
            pRobot->ArmKicker();
        pTimer->Start();
        pTimer->Reset();
        autonState = DRIVE_8_FT;
    }
    // fall through

    case DRIVE_8_FT:
    {
        // TODO: user encoders to measure distance
        pRobot->Drive(DRIVE_SPEED_8FT, DRIVE_SPEED_8FT);
        if (pTimer->Get() >= DRIVE_TIME_8FT)
            autonState = (autonState_t)((int)autonState + 1);
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
                autonState = (autonState_t)((int)autonState + 1);
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
                autonState = (autonState_t)((int)autonState + 1);
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

            autonState = (autonState_t)((int)autonState + 1);
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
            autonState = (autonState_t)((int)autonState + 1);
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
    //--- Get the Robot instance
    Robot980* pRobot = Robot980::GetInstance();

    //--- Get the autonomous mode timer in seconds
    float t = pTimerAuton->Get();

    //--- Drive forward
    if (t < 1.0)
    {
        pRobot->ArmKicker();
        pRobot->Drive(0.4, 0.4, -0.2); // left, right, roller
    }

    //--- Stop and fire
    else if (t < 3.0)
    {
        pRobot->Drive(0, 0, -0.2); // stop

        static bool bFired = false;

        if (! bFired)
        {
            pRobot->FireKicker();
            bFired = true;
        }
    }

    //--- Rearm the kicker and set coasting mode
    else if (t < 6.0)
    {
        pRobot->ArmKicker();
        pRobot->SetBrakes(false);
    }

    //--- Drive back to the starting position
    else if (t < 6.9)
    {
        pRobot->Drive(-0.4, -0.4, -0.2); // left, right, roller
    }

    else
    {
        pRobot->Drive(0,0,0);
    }
}

//==========================================================================
void Auton6(void)
{
    //--- Get the Robot instance
    Robot980* pRobot = Robot980::GetInstance();

    //--- Get the autonomous mode timer in seconds
    float t = pTimerAuton->Get();

    //--- Wait
    if (t < 11)
    {
        pRobot->Drive(0, 0, 0);          // left, right, roller
    }

    //--- Forward
    else if (t < 14)
    {
        pRobot->Drive(0.5, 0.5, -0.2);   // left, right, roller
    }
    
    //--- Pause
    else if (t < 16)
    {
        pRobot->Drive(0, 0, -0.2);       // left, right, roller
    }

    //--- Back
    else if (t < 18.0 /*19*/)
    {
        //pRobot->SetBrakes(false);
        pRobot->Drive(-0.5, -0.5, -0.2); // left, right, roller
    }

    //--- Stop
    else
    {
        pRobot->Drive(0, 0, 0);          // left, right, roller
    }
}
