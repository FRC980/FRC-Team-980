#ifndef MAIN_H
#define MAIN_H

#include <IterativeRobot.h>

class Main : public IterativeRobot
{
  public:
    Main();
    virtual ~Main();

    virtual void RobotInit();

    virtual void DisabledInit();
    virtual void DisabledPeriodic();
    virtual void DisabledContinuous();

    virtual void AutonomousInit();
    virtual void AutonomousPeriodic();
    virtual void AutonomousContinuous();

    virtual void TeleopInit();
    virtual void TeleopPeriodic();
    virtual void TeleopContinuous();
};

#endif // MAIN_H
