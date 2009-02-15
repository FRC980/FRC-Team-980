#include <WPILib.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"

void Main::Autonomous()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();

    while (IsAutonomous() && !IsDisabled())
    {
        pLCD->Printf(DriverStationLCD::kMain_Line6, 1, "Autonomous");

        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();
        Wait(0.01);
    }
}
