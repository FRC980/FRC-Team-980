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
    pRobot->EnableTractionControl(Robot980::TC_LOWPASS);

    Robot980::tractionMode_t tcOld = pRobot->GetTractionControl();
    bool bOldButton = false;

    while (IsOperatorControl() && !IsDisabled())
    {
        GetWatchdog().Feed();

        // left & right top buttons (4 & 5) enable "smart" traction control
        if (jsDrive.GetRawButton(4) || jsDrive.GetRawButton(5))
            pRobot->EnableTractionControl((tcOld = Robot980::TC_SMART));

        // trigger button enables "low pass filter" traction control
        if (jsDrive.GetRawButton(1))
            pRobot->EnableTractionControl((tcOld = Robot980::TC_LOWPASS));

        // lower thumb button (2) temporarily disables all traction control
        if (jsDrive.GetRawButton(2))
        {
            bOldButton = true;
            pRobot->EnableTractionControl(Robot980::TC_OFF);
        }
        else
        {
            if (bOldButton)
            {
                bOldButton = false;
                pRobot->EnableTractionControl(tcOld);
            }
        }

        float x = jsDrive.GetX();
        x = (x > 0) ? x * x : x * x * -1;

        float y = jsDrive.GetY();
        y = (y > 0) ? y * y : y * y * -1;

        float fLeft  = limit(y - x);
        float fRight = limit(y + x);

        // for debugging, use the "Z" axis and buttons 10/11 for drive
        const float z = (1 - jsDrive.GetZ()) / 2;
        if (jsDrive.GetRawButton(10))
            fLeft = fRight = z;
        if (jsDrive.GetRawButton(11))
            fLeft = fRight = -z;

        pRobot->Drive(fLeft, fRight, pLCD);

        float f = jsBelts.GetY();
        // on ball exit, limit belt speed to prevent overshooting
        if (!jsBelts.GetRawButton(1))
        {
            if (f > 0)
                f *= 0.65;
        }
        d.Printf("belts  %f\n", f);
        pRobot->RunBelts(fabs(f), f);

        d.Printf("Auton: %d", pRobot->GetAutonMode());

        pLCD->UpdateLCD();
        DashboardData::UpdateAndSend();
        Wait(0.05);

        while (!NextPeriodReady())
            Wait(0.01);
    }

    printf("leaving Main::OperatorControl()\n");
}
