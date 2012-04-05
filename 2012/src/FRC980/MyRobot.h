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
    CANJaguar *m_pscShooterMaster;
    CANJaguar *m_pscShooterSlave1;

    CANJaguar *m_pscLeft1;
    CANJaguar *m_pscRight1;

    Victor *m_pscBallPickup;
    Victor *m_pscBallFeeder;
    Victor *m_pscBridge;

    MyJoystick *joystick1;
    MyJoystick *joystick2;
    MyJoystick *steeringwheel;

    DriverStation *ds;

    //ADXL345_I2C *m_pAccelerometer;

    Timer *m_bridge_timer;

    // void Rotate(float degrees);

public:
    MyRobot(void);
    ~MyRobot(void);
    void Autonomous(void);
    void OperatorControl(void);
    void RunBridge(bool);
    void CheckStopBridge(void);
    float GetRPM(void);
    void Drive(float, float);
    void Rotate(float);
    void DriveControlPosition(float, float);
    void DriveControlSpeed(float, float);
    void DriveControl(float, float);
    void SetShooterSpeed(float);
    float GetRightEncoder(void);
    float GetLeftEncoder(void);
    void SetBrakes(bool);
    void PerformBalanceTrick(MyJoystick *joy);
    void PerformBalanceTrickSpeed(MyJoystick *joy);
    void DriveControlMode(CANJaguar::ControlMode);
};

#endif
