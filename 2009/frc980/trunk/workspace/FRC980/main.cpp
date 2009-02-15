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

void Main::Init()
{
    // Create a new semaphore
    m_packetDataAvailableSem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);

    // Register that semaphore with the network communications task.
    // It will signal when new packet data is available.
    setNewDataSem(m_packetDataAvailableSem);

    Robot980::GetInstance();

//    GetWatchdog().SetEnabled(true);
    m_ds->GetDashboardPacker();
}

bool Main::NextPeriodReady()
{
    int success = semTake(m_packetDataAvailableSem, 0);

    if (success == OK)
    {
        return true;
    }
    return false;
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
        DriverStationLCD::GetInstance()->Clear();

        while (IsDisabled())
        {
            DriverStationLCD::GetInstance()->Clear();
            Disabled();
            while (IsDisabled())
                Wait(0.01);
        }

        while (IsAutonomous() && !IsDisabled())
        {
            DriverStationLCD::GetInstance()->Clear();
            Autonomous();
            while (IsAutonomous() && !IsDisabled())
                Wait(0.01);
        }

        while (IsOperatorControl() && !IsDisabled())
        {
            DriverStationLCD::GetInstance()->Clear();
            OperatorControl();
            while (IsOperatorControl() && !IsDisabled())
                Wait(0.01);
        }
    }
}

START_ROBOT_CLASS(Main);
