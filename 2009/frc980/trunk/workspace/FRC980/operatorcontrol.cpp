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
    Joystick jsDrive(2);
    Joystick jsBelts(1);

    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();
    Dashboard &d = DriverStation::GetInstance()->GetDashboardPacker();

    printf("in Main::OperatorControl()\n");
    pRobot->EnableTractionControl(false);

    while (IsOperatorControl() && !IsDisabled())
    {
        GetWatchdog().Feed();

        if (jsDrive.GetRawButton(6) || jsDrive.GetRawButton(11) ||
            jsDrive.GetRawButton(4) || jsDrive.GetRawButton(5))
            pRobot->EnableTractionControl(true);
        if (jsDrive.GetRawButton(7) || jsDrive.GetRawButton(10) ||
            jsDrive.GetRawButton(1) || jsDrive.GetRawButton(2))
            pRobot->EnableTractionControl(false);

        float x = jsDrive.GetX();
        x = (x > 0) ? x * x : x * x * -1;

        float y = jsDrive.GetY();
        y = (y > 0) ? y * y : y * y * -1;

        static float fLeft = 0;
        static float fRight = 0;
        float fRateLimit = 0.05;
//        float fRateLimit = 1;

        fLeft  = limit( limit(y - x), fLeft  - fRateLimit, fLeft  + fRateLimit);
        fRight = limit( limit(y + x), fRight - fRateLimit, fRight + fRateLimit);

        pRobot->Drive(fLeft, fRight, pLCD);

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
            float f = jsBelts.GetY();

            // on ball exit, limit belt speed to prevent overshooting
            if (!jsBelts.GetRawButton(1))
            {
                if (f > 0)
                    f *= 0.65;
            }

            d.Printf("belts  %f", f);
            pRobot->RunBelts(fabs(f), f);
        }
        d.Printf("\nAuton: %d", pRobot->GetAutonMode());

        pLCD->UpdateLCD();
        DashboardData::UpdateAndSend();
        Wait(0.05);

        while (!NextPeriodReady())
            Wait(0.01);
    }

    printf("leaving Main::OperatorControl()\n");
}
