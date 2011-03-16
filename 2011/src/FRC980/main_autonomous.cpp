#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==========================================================================

typedef enum 
{
    AUTON_INIT,
    AUTON_CLOSE_CLAW,
    AUTON_RELEASE_CLAW,
    AUTON_DRIVE_FORWARD,
    AUTON_RAISE_ARM,
    AUTON_OPEN_CLAW,
    AUTON_LOWER_ARM,
    AUTON_DRIVE_REVERSE,
    AUTON_DONE
} auton_state_t;

auton_state_t auton_state = AUTON_INIT;

static int iMode = 0;
static Timer *pTimerAuton = new Timer;
static bool goLeft = false;
static bool bStraightLine = true;
static bool bLineTrackModeInitialized = false;
static float encoder_initial;

float GetSpeedStraight(void);
float GetSpeedTurn(void);
float GetSteeringGainStraight(void);
float GetSteeringGainTurn(void);
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
    iMode = 6;

    pRobot->SetBrakes(false);

    switch (iMode)
    {
    case 1:
        goLeft = true;
        bStraightLine = false;
        bLineTrackModeInitialized = false;
        break;
    case 2:
        goLeft = true;
        bStraightLine = true;
        bLineTrackModeInitialized = false;
        break;
    case 3:
        goLeft = false;
        bStraightLine = false;
        bLineTrackModeInitialized = false;
        break;
	case 6:
	    pRobot->SetBrakes(true);
        break;
    }

    encoder_initial = pRobot->GetRightEncoder();
    auton_state = AUTON_INIT;

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

#define LINEAR_RAMP(variable,start_variable,end_variable,start_speed,end_speed) \
    start_speed + (end_speed-start_speed)/(end_variable-start_variable)*(end_variable-variable)

float GetSpeedStraight(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float distance = pRobot->GetRightEncoder() - encoder_initial;
    if (distance < 90.0)
        return 0.25;
    else if (distance < 100.0)
        return LINEAR_RAMP(distance,90.0,100.0,0.25,0);
    else
        return 0.0;
}

float GetSteeringGainStraight(void)
{
    return GetSpeedStraight() * 1.0;
}

float GetSpeedTurn(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float distance = pRobot->GetRightEncoder() - encoder_initial;

    if (distance < 100.0)
        return 0.25;
    else if (distance < 131.4)
        return LINEAR_RAMP(distance,100.0,131.4,0.25,0.15);
    else
        return 0.15;

}

float GetSteeringGainTurn(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float distance = pRobot->GetRightEncoder() - encoder_initial;
    if (distance < 100.0)
        return 0.25;
    else if (distance < 131.4)
        return GetSpeedTurn() * LINEAR_RAMP(distance,100,131.4,1.0,1.5);
    else
        return 0.20;
}

void AutonLineTrack(void)
{
    Robot980 *pRobot = Robot980::GetInstance();
    float t = pTimerAuton->Get();
    float distance = pRobot->GetRightEncoder() - encoder_initial;

    static double stopTime;
    static bool atCross = false;    // true when robot has reached end
    static char previousValue = 0; // previous binary value

    if (!bLineTrackModeInitialized)
    {
        // Print out a message displaying which
        // line tracking mode we chose
        utils::message("Line tracker auton mode");
        utils::message("StraightLine: %d\n", bStraightLine);
        utils::message("GoingLeft: %d\n", goLeft);

        stopTime = bStraightLine ? 2.0 : 4.0;
        atCross = false;
        previousValue=0;

        bLineTrackModeInitialized = true;
    }


    if (atCross)
    {
        //We have reached the cross
        //Now hang the ubertube
        utils::message("At cross");
        return;
    }

    char binaryValue = pRobot->GetLineTracker(! goLeft);    
    double speed = bStraightLine ? GetSpeedStraight() : GetSpeedTurn();
    double steeringGain = bStraightLine ? GetSteeringGainStraight() : GetSteeringGainTurn();
    if (! goLeft) steeringGain *= -1.0;
        // If going left, steer to the right, and vice versa
    double turn = 0;

    switch (binaryValue) {
        // three bits in the format [outer, middle, inner]
    case 1:
        // just the inside sensor - drive straight
        turn = 0;
        break;
    case 7:
        // all sensors - maybe at the "T"
        if (t > stopTime || (pRobot->GetRightEncoder() - encoder_initial > 160.0) ) {
            atCross = true;
            speed = 0;
        }
        break;
    case 0:
        // no sensors
        if (previousValue == 0)
        {
            // If we start the match with no sensor reads!!!
            utils::message("WARNING: no sensor data");
            // apply the steering gain
            turn = steeringGain;
        }
        if (previousValue == 1) {
            // Previously it was just the outside sensor
            // We drove away from the line, so steer back
            turn = steeringGain;
        }
        else {
            // In the rare case that the line is now on the
            // opposite side of the sensors, steer back
            turn = -steeringGain;
        }
        break;
    default:
        // If the line is read by the middle or outside sensors
        turn = -steeringGain;
    }
    
    utils::message("t=%2.2f sensor=%d speed=%1.2f turn=%1.2f distance=%f\n", t, binaryValue, speed, turn, distance);
    
    // move the robot forward
    pRobot->Drive(speed+turn, speed-turn);
    
    if (binaryValue != 0) previousValue = binaryValue;

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

    float target_arm_height = 475;
    float target_distance = 195.0;

    static float initial_state_time = pTimerAuton->Get();

    switch (auton_state)
    {
    case AUTON_INIT:
        auton_state = AUTON_CLOSE_CLAW;
        //fall through
    case AUTON_CLOSE_CLAW:
        pRobot->RunClaw(-0.9);
        if (t > 1.0)
        {
            auton_state = AUTON_RELEASE_CLAW;
            initial_state_time = t;
            pRobot->RunClaw(0.0);
        }
        break;
    case AUTON_RELEASE_CLAW:
        if ( (t - initial_state_time) < 0.2)
        {
            pRobot->Drive(-1.0,-1.0);
        }
        else if ( (t - initial_state_time) < 0.4)
        {
            pRobot->Drive(0.0,0.0);
        }
        else
        {
            auton_state = AUTON_DRIVE_FORWARD;
        }
        break;
    case AUTON_DRIVE_FORWARD:
        pRobot->SetPosition(target_arm_height);
        if (distance < target_distance)
        {
            float speed = 0.40;
            pRobot->Drive(speed,speed);
            utils::message("Distance = %f\n", distance);
        }
        else
        {
            pRobot->Drive(0.0,0.0);
            utils::message("Distance_final = %f\n", distance);
            auton_state = AUTON_RAISE_ARM;
        }
        break;
    case AUTON_RAISE_ARM:
        if (
            ((pRobot->GetPosition() - target_arm_height) < 10)
            || ((pRobot->GetPosition() - target_arm_height) > -10)
            )
        {
            pRobot->SetArmSpeed(0.0);
            auton_state=AUTON_OPEN_CLAW;
            initial_state_time = t;
        }
        else
        {
            utils::message("Arm at %f", pRobot->GetPosition());
        }
        break;
    case AUTON_OPEN_CLAW:
        if ( (t - initial_state_time) > 1.0)
        {
            pRobot->RunClaw(0.0);
            auton_state = AUTON_LOWER_ARM;
            initial_state_time = t;
            utils::message("claw open");
        }
        else
        {
            pRobot->RunClaw(0.9);
            utils::message("closing claw");
        }
        break;
    case AUTON_LOWER_ARM:
        //let gravity do the work
        if ( (t - initial_state_time) > 2.0)
        {
            auton_state = AUTON_DRIVE_REVERSE;
            initial_state_time = t;
            utils::message("driving back");
        }
        break;
    case AUTON_DRIVE_REVERSE:
        if ( (t - initial_state_time) > 2.0)
        {
            pRobot->Drive(0.0, 0.0);
            auton_state = AUTON_DONE;
        }
        else
        {
            pRobot->Drive(-0.25, -0.25);
        }
        break;
    }
}
