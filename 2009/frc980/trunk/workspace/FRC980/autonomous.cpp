#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"
#include "utils.h"

// MODE 1: Drive straight
void Main::Auton1()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();
    pRobot->Drive(-0.6, -0.6, pLCD); // negative is forward
}

// MODE 2: Drive pseudo-randomly
void Main::Auton2()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();

    static double x = 0;
    x += 0.01;
    pRobot->Drive(sin(0.6*x), sin(0.5*x), pLCD);
}

// MODE 3: Spin in circles
void Main::Auton3()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();

    static float fSpeed = 0.1;

    fSpeed = limit(fSpeed + 0.05);

    if (fSpeed < 0.9)
        pRobot->Drive(- fSpeed, - fSpeed, pLCD); // negative is forward
    else
        pRobot->Drive(fSpeed, -fSpeed, pLCD);
}

void Main::Auton4()
{
    // DO STUFF HERE
}

void Main::Auton5()
{
    // DO STUFF HERE
}

void Main::Auton6()
{
    // DO STUFF HERE
}

static int iMode = 0;

void Main::AutonomousInit()
{
    Robot980* pRobot = Robot980::GetInstance();
    iMode = pRobot->GetAutonMode();

    printf("in Main::Autonomous()\n");

    pRobot->EnableTractionControl(Robot980::TC_LOWPASS);
}

void Main::AutonomousPeriodic()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();

    GetWatchdog().Feed();

    switch(iMode)
    {
    default:
    case 1:
        Auton1();
        break;

    case 2:
        Auton2();
        break;

    case 3:
        Auton3();
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

    DashboardData::UpdateAndSend();
    pLCD->UpdateLCD();
}

void Main::AutonomousContinuous()
{}
