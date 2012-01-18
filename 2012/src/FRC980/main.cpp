#include <WPILib.h>

#include "main.h"

#include "Robot980.h"

Main::Main(void)
{
}

Main::~Main(void)
{
}

void Main::RobotInit(void)
{
    //--- Get the Robot instance
    Robot980::GetInstance();

    //--- Set a pointer to the joystick
    Joystick::GetStickForPort(1);

    //--- Set the Watchdog expiration to 100ms
    //    DO NOT CHANGE THIS NUMBER
    GetWatchdog().SetExpiration(100);
}

START_ROBOT_CLASS(Main);
