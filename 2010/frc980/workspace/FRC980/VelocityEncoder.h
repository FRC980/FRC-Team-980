#ifndef VELOCITYENCODER_H
#define VELOCITYENCODER_H

#include <PIDSource.h>
class Encoder;

// This provides the PIDSource interface in the form of an encouder which
// reports velocity (as opposed to a normal encoder which provides the
// PIDSource interface reporting position)
class VelocityEncoder : public PIDSource
{
  public:
    VelocityEncoder(Encoder* pEnc);

    virtual double PIDGet();

  private:
    Encoder* m_pEnc;
};

#endif // VELOCITYENCODER_H
