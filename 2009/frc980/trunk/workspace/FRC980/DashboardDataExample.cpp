#include <SensorBase.h>
#include <WPILib.h>
#include <AxisCamera.h>
#include <PCVideoServer.h>

#include "DashboardData.h"


/**
 * This is a demo program showing the use of the Dashboard data packing
 * class.
 */
class DashboardDataExample : public SimpleRobot
{
    static const UINT32 kAnalogChannels = SensorBase::kAnalogChannels;
    static const UINT32 kAnalogModules  = SensorBase::kAnalogModules;

    RobotDrive myRobot;         // robot drive system
    Joystick stick;             // only joystick
    DashboardData dashboardData;

  public:
    // these must be initialized in the same order as they are declared above.
    DashboardDataExample(void)
        : myRobot(1, 2)
        , stick(1)
    {
        GetWatchdog().SetExpiration(100);
    }

    /*
     * Runs the motors with arcade steering.
     */
    void RobotMain(void)
    {
        GetWatchdog().SetEnabled(true);
        Dashboard& dashboard = m_ds->GetDashboardPacker();
        INT32 i = 0;

        /* start the CameraTask  */
        StartCameraTask(20, 0, k320x240, ROT_180);
        PCVideoServer *pVideoServer = new PCVideoServer;

        // "Gyro can only be used with Analog Channel 1 on either module"
        // in InitGyro() in Gyro.cpp at line 27
        Gyro gyro(/*slot*/ 1, /*channel*/ 1);
        gyro.SetSensitivity(0.007);
        Wait(0.02);
        gyro.Reset();

        Encoder encoder(/*slot-A*/ 4, /*channel-A*/ 1,
                        /*slot-B*/ 4, /*channel-B*/ 2, /*reverse*/ false);
        encoder.Start();

        while (true)
        {
            GetWatchdog().Feed();
            myRobot.ArcadeDrive(stick);
            dashboard.Printf("It's been %f seconds, according to the FPGA.\n",
                             GetClock());
            dashboard.Printf("Iterations: %d\n", ++i);
            dashboard.Printf("Angle: %f\n", gyro.GetAngle());
            dashboard.Printf("Ticks: %d,  Period: %f\n", encoder.Get(),
                             encoder.GetPeriod());
            UpdateDashboard();
            Wait(0.02);
        }

        delete pVideoServer;
    }

    /*
     * Send data to the dashboard
     * Just sending a few values to show the data changing.
     * These values could be read from hardware.
     */
    void UpdateDashboard(void)
    {
        dashboardData.UpdateAndSend();
    }
};

START_ROBOT_CLASS(DashboardDataExample);
