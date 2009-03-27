#include <WPILib.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"

Main::Main()
{
}

Main::~Main()
{
}

void Main::RobotInit()
{
    DriverStationLCD::GetInstance();
    Robot980::GetInstance();
    DriverStation::GetInstance()->GetDashboardPacker();

    Joystick::GetStickForPort(1);
    Joystick::GetStickForPort(2);
    Joystick::GetStickForPort(3);

    GetWatchdog().SetExpiration(100);
    m_ds->GetDashboardPacker();
}

START_ROBOT_CLASS(Main);
