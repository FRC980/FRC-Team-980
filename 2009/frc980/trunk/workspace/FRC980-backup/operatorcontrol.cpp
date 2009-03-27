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
    Joystick jsDebug(3);
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
        if (jsDrive.GetRawButton(4))
            pRobot->EnableTractionControl((tcOld = Robot980::TC_SMART_1));

        if (jsDrive.GetRawButton(5))
            pRobot->EnableTractionControl((tcOld = Robot980::TC_SMART_2));

        // lower thumb button enables "low pass filter" traction control
        if (jsDrive.GetRawButton(2))
            pRobot->EnableTractionControl((tcOld = Robot980::TC_LOWPASS));

        // trigger button (1) temporarily disables all traction control
        if (jsDrive.GetRawButton(1))
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
        const float z = (1 - jsDebug.GetZ()) / 2;
        if (jsDebug.GetRawButton(10))
            fLeft = fRight = z;
        if (jsDebug.GetRawButton(11))
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
        Wait(0.020);

        while (!NextPeriodReady())
            Wait(0.001);
    }

    printf("leaving Main::OperatorControl()\n");
}
