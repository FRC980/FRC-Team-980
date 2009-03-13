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

    float fSpeed = 0.1;
    while (IsAutonomous() && !IsDisabled())
    {
        GetWatchdog().Feed();
        fSpeed = limit(fSpeed + 0.05);
        pRobot->Drive(fSpeed, fSpeed, pLCD);

        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();

        while (!NextPeriodReady())
            Wait(0.01);
    }
}

// MODE 2: Drive pseudo-randomly
void Main::Auton2()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();

    double x = 0;
    while (IsAutonomous() && !IsDisabled())
    {
        x += 0.01;

        pRobot->Drive(sin(0.6*x), sin(0.5*x), pLCD);

        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();

        while (!NextPeriodReady())
            Wait(0.01);
    }
}

// MODE 3: Spin in circles
void Main::Auton3()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();

    float fSpeed = 0.1;
    while (IsAutonomous() && !IsDisabled())
    {
        GetWatchdog().Feed();
        fSpeed = limit(fSpeed + 0.05);

        if (fSpeed < 0.9)
            pRobot->Drive(fSpeed, fSpeed, pLCD);
        else
            pRobot->Drive(fSpeed, -fSpeed, pLCD);

        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();

        while (!NextPeriodReady())
            Wait(0.01);
    }
}

void Main::Auton4()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();

    while (IsAutonomous() && !IsDisabled())
    {
        GetWatchdog().Feed();

        // DO STUFF HERE

        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();

        while (!NextPeriodReady())
            Wait(0.01);
    }
}

void Main::Auton5()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();

    while (IsAutonomous() && !IsDisabled())
    {
        GetWatchdog().Feed();

        // DO STUFF HERE

        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();

        while (!NextPeriodReady())
            Wait(0.01);
    }
}

void Main::Auton6()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();

    while (IsAutonomous() && !IsDisabled())
    {
        GetWatchdog().Feed();

        // DO STUFF HERE

        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();

        while (!NextPeriodReady())
            Wait(0.01);
    }
}

void Main::Autonomous()
{
    Robot980* pRobot = Robot980::GetInstance();
    int iMode = pRobot->GetAutonMode();

    printf("in Main::Autonomous()\n");

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
}
