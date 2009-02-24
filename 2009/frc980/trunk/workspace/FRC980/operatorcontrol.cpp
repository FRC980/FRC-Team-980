#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"

float limit(float val, float min = -1, float max = 1)
{
    if (val > max)
        return max;

    if (val < min)
        return min;

    return val;
}

void Main::OperatorControl()
{
    Joystick jsDrive(1);
    Joystick jsBelts(2);

    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();
    Dashboard &d = DriverStation::GetInstance()->GetDashboardPacker();

    printf("in Main::OperatorControl()\n");
    pRobot->EnableTractionControl(false);

    while (IsOperatorControl() && !IsDisabled())
    {
        GetWatchdog().Feed();

        if (jsDrive.GetRawButton(6) || jsDrive.GetRawButton(11))
            pRobot->EnableTractionControl(true);
        if (jsDrive.GetRawButton(7) || jsDrive.GetRawButton(10))
            pRobot->EnableTractionControl(false);

        float x = jsDrive.GetX();
        x = (x > 0) ? x * x : x * x * -1;

        float y = jsDrive.GetY();
        y = (y > 0) ? y * y : y * y * -1;

        pRobot->Drive(limit(y - x), limit(y + x), pLCD);

        float z = (1 - jsDrive.GetZ()) / 2;
        if (jsDrive.GetRawButton(8) || jsDrive.GetRawButton(9))
        {
            if (jsDrive.GetRawButton(8))
            {
                pRobot->RunBelts(z, -z);
                d.Printf("In  %f", z);
            }
            if (jsDrive.GetRawButton(9))
            {
                pRobot->RunBelts(z, z);
                d.Printf("Out %f", z);
            }
        }
        else
        {
            d.Printf("Z   %f", z);
            pRobot->RunBelts(fabs(jsBelts.GetY()), jsBelts.GetY());
        }


        pLCD->UpdateLCD();
        DashboardData::UpdateAndSend();
        Wait(0.05);

        while (!NextPeriodReady())
            Wait(0.01);
    }

    printf("leaving Main::OperatorControl()\n");
}
