#ifndef SMARTDRIVE_H
#define SMARTDRIVE_H

#include <SpeedController.h>

class Encoder;

class SmartDrive : public SpeedController
{
  public:
    SmartDrive(SpeedController* psc, Encoder* pEncDrive, Encoder* pEncFollow);

    virtual void Set(float speed);
    virtual float Get();

  private:
    SpeedController*  m_psc;
    Encoder* m_pEncDrive;
    Encoder* m_pEncFollow;
};

#endif // SMARTDRIVE_H
