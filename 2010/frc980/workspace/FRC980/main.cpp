#include <WPILib.h>

#include "main.h"

#include "Robot980.h"

Main::Main()
{
}

Main::~Main()
{
}

void Main::RobotInit()
{
    Robot980::GetInstance();

    Joystick::GetStickForPort(1);

    GetWatchdog().SetExpiration(250);
}

START_ROBOT_CLASS(Main);
