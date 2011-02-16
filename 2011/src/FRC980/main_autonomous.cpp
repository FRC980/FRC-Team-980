#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==========================================================================
static int iMode = 0;
static Timer *pTimerAuton = new Timer;

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
    switch (iMode)
    {
    case 1:  Auton1();  break;
    case 2:  Auton2();  break;
    case 3:  Auton3();  break;
    case 4:  Auton4();  break;
    case 5:  Auton5();  break;
    default:
    case 6:  Auton6();  break;
    }
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
}
