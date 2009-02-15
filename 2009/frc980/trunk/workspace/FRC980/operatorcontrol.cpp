#include <WPILib.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"

void Main::OperatorControl()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();

    while (IsOperatorControl() && !IsDisabled())
    {
        pLCD->Printf(DriverStationLCD::kMain_Line6, 1, "OperatorControl");

        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();

        while (!NextPeriodReady())
            Wait(0.001);
    }
}
