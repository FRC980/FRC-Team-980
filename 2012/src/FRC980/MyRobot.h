#include "WPILib.h"

#ifndef _MyRobot_h_
#define _MyRobot_h_

#define Jaguar1     1

class MyRobot : public SimpleRobot
{
    Jaguar *jag1;
    Joystick *joystick1;
    SteeringWheel *SteeringWheel2;
    DriverStation *ds;

public:
    MyRobot(void);
    ~MyRobot(void);
    void Autonomous(void);
    void OperatorControl(void);
    void Drive(float); 
};

#endif
