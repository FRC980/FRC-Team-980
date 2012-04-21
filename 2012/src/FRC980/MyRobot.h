#include "WPILib.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Accelerometer.h"
#include "MyJoystick.h"

#ifndef _MyRobot_h_
#define _MyRobot_h_

#define Jaguar1     1

class MyRobot : public SimpleRobot
{
private:
    CANJaguar *m_pscLeft1;
    CANJaguar *m_pscRight1;

    MyJoystick *joystick1;
    MyJoystick *steeringwheel;

    DriverStation *ds;

public:
    MyRobot(void);
    ~MyRobot(void);
    void Autonomous(void);
    void OperatorControl(void);
    void Drive(float, float);
    void DriveControlPosition(float, float);
    void DriveControlSpeed(float, float);
    void DriveControl(float, float);
    float GetRightEncoder(void);
    float GetLeftEncoder(void);
    float GetSpeedRight(void);
    float GetSpeedLeft(void);
    void SetBrakes(bool);
    void DriveControlMode(CANJaguar::ControlMode);
    int GetAutonMode(void);
};

#endif
