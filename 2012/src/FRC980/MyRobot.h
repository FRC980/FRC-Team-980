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
    CANJaguar *m_pscShooterSlave2;
    CANJaguar *m_pscShooterSlave3;

    CANJaguar *m_pscLeft1;
    CANJaguar *m_pscLeft2;
    CANJaguar *m_pscRight1;
    CANJaguar *m_pscRight2;

    Victor *m_pscBallPickup;
    Victor *m_pscBallFeeder;
    Victor *m_pscTurret;

    MyJoystick *joystick1;
    MyJoystick *joystick2;
    MyJoystick *steeringwheel;

    DriverStation *ds;

    Accelerometer *accelerometer;

public:
    MyRobot(void);
    ~MyRobot(void);
    void Autonomous(void);
    void OperatorControl(void);
    float GetRPM(void);
    void Drive(float, float); 
    void SetShooterSpeed(float);
    float GetRightEncoder(void);
    float GetLeftEncoder(void);
    void SetBrakes(bool);
    void PerformBalanceTrick(MyJoystick *joy);
};

#endif
