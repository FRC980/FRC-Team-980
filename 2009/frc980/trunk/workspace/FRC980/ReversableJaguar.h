#ifndef REVERSABLEJAGUAR_H
#define REVERSABLEJAGUAR_H

#include <Jaguar.h>

class ReversableJaguar : public Jaguar
{
  public:
    explicit ReversableJaguar(UINT32 channel, bool bReversed = true);
    ReversableJaguar(UINT32 slot, UINT32 channel, bool bReversed = true);
    virtual ~ ReversableJaguar() {};

    virtual float Get();
    virtual void Set(float value);

    bool GetReversed() { return m_bReversed; };
    void SetReversed(bool bReversed = true)
        { m_bReversed = bReversed; };

  private:
    bool m_bReversed;
};

#endif // REVERSABLEJAGUAR_H
