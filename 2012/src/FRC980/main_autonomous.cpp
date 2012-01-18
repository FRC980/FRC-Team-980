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

    switch (iMode)
    {
    case 1:
    case 2:
    case 3:
    case 4:
	case 6:
		break;
    }

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
#ifdef DEBUG_AUTON
    case 1:
    case 2:
    case 3:
    case 4:
    case 5:
        break;
#else
    case 1:
    case 2:
    case 3:
    case 4: Auton4(); break;
    case 5: Auton5(); break;
#endif
    default:
    case 6:  Auton6();  break;
    }
}

//==========================================================================
void Auton1(void)
{
}

//==========================================================================
void Auton2(void)
{
}

//==========================================================================
void Auton3(void)
{
}

//==========================================================================
void Auton4(void)
{
}

//==========================================================================
void Auton5(void)
{
}

//==========================================================================
void Auton6(void)
{
}
