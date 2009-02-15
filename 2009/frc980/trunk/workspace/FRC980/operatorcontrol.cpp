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

    while (IsOperatorControl() && !IsDisabled())
    {
        if (jsDrive.GetRawButton(6))
            pRobot->EnableTractionControl(true);
        if (jsDrive.GetRawButton(7))
            pRobot->EnableTractionControl(false);

        float x = jsDrive.GetX();
        x = (x > 0) ? x * x : x * x * -1;

        float y = jsDrive.GetY();
        y = (y > 0) ? y * y : y * y * -1;

        pRobot->Drive(limit(y - x), limit(y + x), pLCD);
        pRobot->RunBelts(fabs(jsBelts.GetY()), jsBelts.GetY());


        DashboardData::UpdateAndSend();
        pLCD->UpdateLCD();

        while (!NextPeriodReady())
            Wait(0.001);
    }
}
