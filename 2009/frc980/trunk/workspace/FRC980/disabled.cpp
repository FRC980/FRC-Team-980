#include <WPILib.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"

void Main::Disabled()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();

    printf("in Main::Disabled()\n");

    while (IsDisabled())
    {
        GetWatchdog().Feed();
        pLCD->Printf(DriverStationLCD::kMain_Line6, 1, "Disabled");

//        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();
        Wait(0.01);
    }
}
