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
    
    //--- NOTE: THIS IS A STOPGAP UNTIL A WAY TO CHOOSE THIS MODE IS MADE
    //    ALSO IT ALLOWS US TO START UTILIZING MORE THAN ONE AUTON MODE
    iMode = 1;

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

//==============================================================================
//==============================================================================
void Main::Auton1()
{
    float t = pTimerAuton->Get();

    //--- In the first two seconds of the match drive forward
    if (t < 2)
    {
        pRobot->Drive(0.5, 0.5, 0.2); // left, right, roller
    }

    //--- After two seconds stop the robot and fire
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

    //--- After two and a half seconds rearm the kicker
    if (t > 2.5)
    {
        pRobot->ArmKicker();
    }
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

