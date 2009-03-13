
#ifndef MAIN_H
#define MAIN_H

#include <RobotBase.h>
#include <Timer.h>

class Main : public RobotBase
{
  public:
    Main();
    virtual ~Main();

    virtual void Init();
    virtual void Disabled();
    virtual void Autonomous();
    virtual void OperatorControl();
    virtual void StartCompetition();
    bool NextPeriodReady();

    void Auton1();
    void Auton2();
    void Auton3();
    void Auton4();
    void Auton5();
    void Auton6();

  private:
    SEM_ID m_packetDataAvailableSem;
};

#endif // MAIN_H
