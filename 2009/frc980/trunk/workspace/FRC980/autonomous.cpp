#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"

void Main::Auton1()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();


    while (IsAutonomous() && !IsDisabled())
    {


        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();
        Wait(0.01);
    }
}

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
        Wait(0.01);
    }
}

void Main::Autonomous()
{
  int prog = 2;

  switch(prog)
  {
  default:
  case 1:
      Auton1();
      break;

  case 2:
      Auton2();
      break;
    }
}
