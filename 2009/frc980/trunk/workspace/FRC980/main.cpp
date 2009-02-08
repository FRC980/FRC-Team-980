#include <Timer.h>

#include "main.h"

Main::Main()
{
}

Main::~Main()
{
}

void Main::Init()
{
}

void Main::Disabled()
{
}

void Main::Autonomous()
{
}

void Main::OperatorControl()
{
}

/*
 * Start a competition -- this is very similar to
 * SimpleRobot::StartCompetition() from WPILib, but adds a call to
 * Disabled();
 */
void Main::StartCompetition()
{
    Init();

    while (1)
    {
        while (IsDisabled())
        {
            Disabled();
            while (IsDisabled())
                Wait(0.01);
        }

        while (IsAutonomous() && !IsDisabled())
        {
            Autonomous();
            while (IsAutonomous() && !IsDisabled())
                Wait(0.01);
        }

        while (IsOperatorControl() && !IsDisabled())
        {
            OperatorControl();
            while (IsOperatorControl() && !IsDisabled())
                Wait(0.01);
        }
    }
}

//START_ROBOT_CLASS(Main);
