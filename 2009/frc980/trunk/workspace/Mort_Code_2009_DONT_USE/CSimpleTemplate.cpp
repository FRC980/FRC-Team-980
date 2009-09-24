#include "MORT_includes.h"
#include "MORT_defines.h"

void Initialize(void)
{
    (StartCameraTask() == -1)
        ? printf("Failed to spawn camera task: %s\n",
                 GetVisionErrorText(GetLastVisionError()))
        : printf("INITIALIZED\n");

    SetWatchdogExpiration(300);
}

void Autonomous(void)
{
    SetWatchdogEnabled(false);

    while (IsAutonomous())
    {
        _________(__________());
        SetVictorSpeed(ROLLER_MOTOR, 1.0);
    }
}

void OperatorControl(void)
{
    SetWatchdogEnabled(true);

    while (IsOperatorControl())
    {
        ___(__(LEFT_SIDE), __(RIGHT_SIDE));
        _();
        WatchdogFeed();
    }
}

START_ROBOT_CLASS(SimpleCRobot);
