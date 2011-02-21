#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==========================================================================
static int iMode = 0;
static Timer *pTimerAuton = new Timer;
static bool goLeft = false;
static bool bStraightLine = true;
static bool bLineTrackModeInitialized = false;
static float encoder_initial;

void AutonLineTrack(void);
void Auton1(void);
void Auton2(void);
void Auton3(void);
void Auton4(void);
void Auton5(void);
void Auton6(void);

//==========================================================================
void Main::AutonomousInit(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    iMode = pRobot->GetAutonMode();

    pRobot->SetBrakes(false);

    switch (iMode)
    {
    case 1:
        goLeft = true;
        bLineTrackModeInitialized = false;
        break;
    case 2:
        goLeft = false;
        bLineTrackModeInitialized = false;
        break;
    }

    encoder_initial = pRobot->GetRightEncoder();

    utils::message("Running autonomous mode %d\n", iMode);

    pTimerAuton->Start();
    pTimerAuton->Reset();
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
    switch (iMode)
    {
    case 1:
    case 2:
    case 3:
        AutonLineTrack();
        break;
    case 4:  Auton4();  break;
    case 5:  Auton5();  break;
    default:
    case 6:  Auton6();  break;
    }
}


//==========================================================================
void AutonLineTrack(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float time = pTimerAuton->Get();
    
    static double forkProfile[] = {0.07, 0.07, 0.06, 0.06, 0.06, 0.05, 0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static double straightProfile[] = {0.5, 0.5, 0.35, 0.35, 0.25, 0.25, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    static double stopTime;
    static double* powerProfile;
    static bool atCross;    // true when robot has reached end

    int timeInSeconds = (int) time;
    static int oldTimeInSeconds = -1;

    double steeringGain = 0.65;
    
    char binaryValue = pRobot->GetLineTracker(! goLeft);
    static char previousValue=0;

    if (!bLineTrackModeInitialized)
    {
        // Print out a message displaying which
        // line tracking mode we chose
        utils::message("Line tracker auton mode");
        utils::message("StraightLine: %d\n", bStraightLine);
        utils::message("GoingLeft: %d\n", goLeft);

        stopTime = bStraightLine ? 2.0 : 4.0;
        powerProfile = bStraightLine ? straightProfile : forkProfile;
        atCross = false;

        oldTimeInSeconds = -1;

        previousValue=0;

        bLineTrackModeInitialized = true;
    }
    
    double speed, turn;

    if (! goLeft)
    {
        // flip the sensors and the steering gain
        steeringGain *= -1;
    }

    speed = powerProfile[timeInSeconds];    // speed value for this time
    turn = 0;                               // default to no turn

    switch (binaryValue) {
    case 1:
        // just the outside sensor - drive straight
        turn = 0;
        break;
    case 7:
        // all sensors - maybe at the "T"
        if (time> stopTime) {
            atCross = true;
            speed = 0;
        }
        break;
    case 0:
        // no sensors - apply previous correction
        if (previousValue == 0 || previousValue == 1) {
            turn = steeringGain;
        }
        else {
            turn = -steeringGain;
        }
        break;
    default:
        // anything else, steer back to the line
        turn = -steeringGain;
    }
    // useful debugging output for tuning your power profile and steering gain
//    if(binaryValue != previousValue)
        utils::message("Time: %2.2f sensor: %d speed: %1.2f turn: %1.2f atCross: %d\n", time, binaryValue, speed, turn, atCross);
    // move the robot forward
    pRobot->Drive(speed+turn, speed-turn);
    if (binaryValue != 0) previousValue = binaryValue;

    oldTimeInSeconds = timeInSeconds;
    Wait(0.01);
}


//==========================================================================
void Auton1(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float t = pTimerAuton->Get();
}

//==========================================================================
void Auton2(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float t = pTimerAuton->Get();
}

//==========================================================================
void Auton3(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float t = pTimerAuton->Get();
}

//==========================================================================
void Auton4(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float t = pTimerAuton->Get();
}

//==========================================================================
void Auton5(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float t = pTimerAuton->Get();
}

//==========================================================================
void Auton6(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float t = pTimerAuton->Get();

    float distance = pRobot->GetRightEncoder() - encoder_initial;

    if (distance < 10.0)
    {
        pRobot->Drive(0.15,0.15);
        utils::message("Distance = %f\n", distance);
    }
    else
    {
        pRobot->Drive(0.0,0.0);
    }
}
