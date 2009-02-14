#include "WPILib.h"

#include "DriverStationLCD.h"

/**
 * This example shows how you can write text to the LCD on the driver station.
 */
class DriverStationLCDTextExample:public SimpleRobot
{

  public:
    DriverStationLCDTextExample(void) { };

    void RobotMain()
    {
        DriverStationLCD *dsLCD = DriverStationLCD::GetInstance();

        while (1)
        {
            dsLCD->Printf(DriverStationLCD::kMain_Line6, 6, "Hello World");
            dsLCD->Printf(DriverStationLCD::kUser_Line2, 8, ":%7.1f",
                          GetClock());
            dsLCD->Printf(DriverStationLCD::kUser_Line3, 17, "%7.1f",
                          GetClock());
            dsLCD->Printf(DriverStationLCD::kUser_Line4, 7, "%7.1f",
                          GetClock());
            dsLCD->Printf(DriverStationLCD::kUser_Line5, 4, "%7.1f",
                          GetClock());
            dsLCD->Printf(DriverStationLCD::kUser_Line6, 10, "%7.1f",
                          GetClock());
            dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "%-7.1f",
                          GetClock());
            dsLCD->UpdateLCD();

            Wait(0.1);
        }
    }
};

START_ROBOT_CLASS(DriverStationLCDTextExample);
