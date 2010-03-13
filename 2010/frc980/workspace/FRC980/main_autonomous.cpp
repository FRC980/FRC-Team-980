#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "Robot980.h"
#include "utils.h"

//==============================================================================
//==============================================================================
static int iMode = 0;
static Timer *pTimerAuton = new Timer;

//==============================================================================
//==============================================================================
void Main::AutonomousInit(void)
{
    Robot980* pRobot = Robot980::GetInstance();
    iMode = pRobot->GetAutonMode();

    pTimerAuton->Start();
    pTimerAuton->Reset();
}

//==============================================================================
void Main::AutonomousContinuous(void)
{
    
}

//==============================================================================
void Main::AutonomousPeriodic(void)
{
    //--- Get the Robot instance
    Robot980* pRobot = Robot980::GetInstance();

    //--- Feed the watchdog
    GetWatchdog().Feed();

    float t = pTimerAuton->Get();

    if (t < 2)
    {
        pRobot->Drive(0.5, 0.5, 0.2); // left, right, roller
    }

    if (t > 2)
    {
        pRobot->Drive(0, 0, 0); // stop

        static bool bFired = false;

        if (! bFired)
        {
            pRobot->FireKicker();
            bFired = true;
        }
    }

    if (t > 2.5)
    {
        pRobot->ArmKicker();
    }
}

//==============================================================================
//==============================================================================
void Main::Auton1()
{
	
}

//==============================================================================
void Main::Auton2()
{
	
}

//==============================================================================
void Main::Auton3()
{
	
}

//==============================================================================
void Main::Auton4()
{
	
}

//==============================================================================
void Main::Auton5()
{
	
}

//==============================================================================
void Main::Auton6()
{
	
}

