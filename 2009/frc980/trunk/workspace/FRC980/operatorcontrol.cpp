#include <WPILib.h>
#include <math.h>

#include "main.h"

#include "DashboardData.h"
#include "DriverStationLCD.h"
#include "Robot980.h"
#include "utils.h"
    
static Robot980::tractionMode_t tcOld = Robot980::TC_LOWPASS;
static bool bOldButton = false;

void Main::TeleopInit()
{
    Robot980* pRobot = Robot980::GetInstance();
    pRobot->EnableTractionControl(Robot980::TC_LOWPASS);

    tcOld = pRobot->GetTractionControl();
}

void Main::TeleopContinuous()
{}

void Main::TeleopPeriodic()
{
    DriverStationLCD* pLCD = DriverStationLCD::GetInstance();
    Robot980* pRobot = Robot980::GetInstance();
    Dashboard &d = DriverStation::GetInstance()->GetDashboardPacker();

    Joystick* pjsDebug = Joystick::GetStickForPort(3);
    Joystick* pjsDrive = Joystick::GetStickForPort(2);
    Joystick* pjsBelts = Joystick::GetStickForPort(1);

    GetWatchdog().Feed();

    // left & right top buttons (4 & 5) enable "smart" traction control
    if (pjsDrive->GetRawButton(4))
        pRobot->EnableTractionControl((tcOld = Robot980::TC_SMART_1));

    if (pjsDrive->GetRawButton(5))
        pRobot->EnableTractionControl((tcOld = Robot980::TC_SMART_2));

    // lower thumb button enables "low pass filter" traction control
    if (pjsDrive->GetRawButton(2))
        pRobot->EnableTractionControl((tcOld = Robot980::TC_LOWPASS));

    // trigger button (1) temporarily disables all traction control
    if (pjsDrive->GetRawButton(1))
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

    float x = pjsDrive->GetX();
    x = (x > 0) ? x * x : x * x * -1;

    float y = pjsDrive->GetY();
    y = (y > 0) ? y * y : y * y * -1;

    float fLeft  = limit(y - x);
    float fRight = limit(y + x);

    // for debugging, use the "Z" axis and buttons 10/11 for drive
    const float z = (1 - pjsDebug->GetZ()) / 2;
    if (pjsDebug->GetRawButton(10))
        fLeft = fRight = z;
    if (pjsDebug->GetRawButton(11))
        fLeft = fRight = -z;

    pRobot->Drive(fLeft, fRight, pLCD);

    float f = pjsBelts->GetY();
    // on ball exit, limit belt speed to prevent overshooting
    if (!pjsBelts->GetRawButton(1))
    {
        if (f > 0)
            f *= 0.65;
    }
    d.Printf("belts  %f\n", f);
    pRobot->RunBelts(fabs(f), f);

    d.Printf("Auton: %d", pRobot->GetAutonMode());

    pLCD->UpdateLCD();
    DashboardData::UpdateAndSend();
}
