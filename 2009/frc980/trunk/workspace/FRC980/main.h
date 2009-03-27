#ifndef MAIN_H
#define MAIN_H

#include <IterativeRobot.h>
#include <Timer.h>

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
    
//    virtual void Init();
//    virtual void Disabled();
//    virtual void Autonomous();
//    virtual void OperatorControl();

//    virtual void StartCompetition();

//    bool NextPeriodReady();

    void Auton1();
    void Auton2();
    void Auton3();
    void Auton4();
    void Auton5();
    void Auton6();
};

#endif // MAIN_H
