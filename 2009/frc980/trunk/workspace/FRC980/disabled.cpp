#include <WPILib.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"


void Main::DisabledInit()
{}

void Main::DisabledPeriodic()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Dashboard &d = DriverStation::GetInstance()->GetDashboardPacker();

    static int i = 0;

    GetWatchdog().Feed();
    pLCD->Printf(DriverStationLCD::kMain_Line6, 1, "Disabled");
    d.Printf("Disabled %d\n", i++);

    DashboardData::UpdateAndSend();
    pLCD->UpdateLCD();
}

void Main::DisabledContinuous()
{}
