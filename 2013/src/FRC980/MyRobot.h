#include "WPILib.h"

#ifndef MyRobot_h_
#define MyRobot_h_

class MyRobot : public SimpleRobot
{
private:
    
public:
    MyRobot(void);
    ~MyRobot(void);
    void Autonomous(void);
    void OperatorControl(void);
};

#endif
