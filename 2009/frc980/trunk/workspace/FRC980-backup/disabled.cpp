#include <WPILib.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"

void Main::Disabled()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Dashboard &d = DriverStation::GetInstance()->GetDashboardPacker();

    printf("in Main::Disabled()\n");

    int i = 0;

    while (IsDisabled())
    {
        GetWatchdog().Feed();
        pLCD->Printf(DriverStationLCD::kMain_Line6, 1, "Disabled");
        d.Printf("Disabled %d\n", i++);

        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();
        while (!NextPeriodReady())
            Wait(0.01);
    }
}
