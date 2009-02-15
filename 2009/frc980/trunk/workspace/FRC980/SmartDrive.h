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
               float period = 0.100);
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

    double m_dCmdSpeed;
    bool  m_bEnabled;

    SpeedController* m_psc;
    Encoder* m_pEncDrive;
    Encoder* m_pEncFollow;

    Notifier *m_controlLoop;
    static void CallCalculate(void *controller);
    void Calculate();

    DISALLOW_COPY_AND_ASSIGN(SmartDrive);
};

#endif // SMARTDRIVE_H
