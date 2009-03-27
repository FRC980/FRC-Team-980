#ifndef VELOCITYENCODER_H
#define VELOCITYENCODER_H

#include <PIDSource.h>
class Encoder;

class VelocityEncoder : public PIDSource
{
  public:
    VelocityEncoder(Encoder* pEnc);

    virtual double PIDGet();

  private:
    Encoder* m_pEnc;
};

#endif // VELOCITYENCODER_H
