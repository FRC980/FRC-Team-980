#ifndef SMARTDRIVE_H
#define SMARTDRIVE_H

#include <SpeedController.h>

class Encoder;

class SmartDrive : public SpeedController
{
  public:
    SmartDrive(double kvp, double kvi, double kvd, // velocity PID constants
               double kcp, double kci, double kcd, // correction PID constants
               double kap, double kai, double kad, // accel PID constants
               SpeedController* psc, Encoder* pEncDrive, Encoder* pEncFollow,
               float period = 0.010);
    virtual ~SmartDrive();

    virtual void Set(float speed) { m_dCmdSpeed = speed; };
    virtual float Get() { return m_dCmdSpeed; };

    void Enable();
    void Disable();
    void Reset();

  private:
    const double m_kVelP, m_kVelI, m_kVelD;
    const double m_kCorP, m_kCorI, m_kCorD;
    const double m_kAclP, m_kAclI, m_kAclD;
    const float  m_kfPeriod;

    double m_dCmdSpeed;
    bool   m_bEnabled;

    SpeedController* m_psc;
    Encoder* m_pEncDrive;
    Encoder* m_pEncFollow;

    Notifier *m_controlLoop;
    static void CallCalculate(void *controller);
    void Calculate();

    double m_dPrevMotorVel;     // previous motor velocity
    double m_dPrevRobotVel;     // previous follow-wheel velocity
    double m_dVelInt;           // integral term in velocity PID loop
    double m_dCorInt;           // integral term in correction PID loop
    double m_dAclInt;           // integral term in acceleration PID loop


    DISALLOW_COPY_AND_ASSIGN(SmartDrive);
};

#endif // SMARTDRIVE_H
