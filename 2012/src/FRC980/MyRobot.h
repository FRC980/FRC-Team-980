#include "WPILib.h"

#ifndef _MyRobot_h_
#define _MyRobot_h_

#define Jaguar1     1

class MyRobot : public SimpleRobot
{
private:
    CANJaguar *m_pscShooter;

    CANJaguar *m_pscLeft1;
    CANJaguar *m_pscLeft2;
    CANJaguar *m_pscRight1;
    CANJaguar *m_pscRight2;

    Joystick *joystick1;
    SteeringWheel *SteeringWheel2;

    DriverStation *ds;

public:
    MyRobot(void);
    ~MyRobot(void);
    void Autonomous(void);
    void OperatorControl(void);
    float GetRPM(void);
    void Drive(float, float); 
};

#endif
