#include <SensorBase.h>
#include <WPILib.h>

#include "DashboardDataFormat.h"


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
    DashboardDataFormat dashboardDataFormat;
    AnalogChannel *m_analogs[kAnalogModules][kAnalogChannels];


  public:
    // these must be initialized in the same order as they are declared above.
    DashboardDataExample(void)
        : myRobot(1, 2)
        , stick(1)
    {
        GetWatchdog().SetExpiration(100);


        for (UINT32 module = 0; module < kAnalogModules; module++)
        {
            for (UINT32 channel = 0; channel < kAnalogChannels; channel++)
            {
                m_analogs[module][channel] = new AnalogChannel(module,channel);
            }
        }
    }

    /*
	 * Runs the motors with arcade steering.
	 */
    void RobotMain(void)
    {
        GetWatchdog().SetEnabled(true);
        Dashboard & dashboard = m_ds->GetDashboardPacker();
        INT32 i = 0;
        while (true)
        {
            GetWatchdog().Feed();
            //myRobot.ArcadeDrive(stick); // drive with arcade style (use right stick)
            dashboard.Printf("It's been %f seconds, according to the FPGA.\n",
                             GetClock());
            dashboard.Printf("Iterations: %d\n", ++i);
            UpdateDashboard();
            Wait(0.02);
        }
    }

    /*
	 * Send data to the dashboard
	 * Just sending a few values to show the data changing.
	 * These values could be read from hardware.
	 */
    void UpdateDashboard(void)
    {
        static float num = 0.0;

        for (UINT32 module = 0; module < kAnalogModules; module++)
        {
            for (UINT32 channel = 0; channel < kAnalogChannels; channel++)
            {
                dashboardDataFormat.m_AnalogChannels[module][channel] =
                    m_analogs[module][channel]->GetValue();
            }
        }

        dashboardDataFormat.m_DIOChannels[0]++;
        dashboardDataFormat.m_DIOChannelsOutputEnable[0]--;
        num += 0.01;
        if (num > 5.0)
            num = 0.0;
        dashboardDataFormat.PackAndSend();
    }
};

START_ROBOT_CLASS(DashboardDataExample);
