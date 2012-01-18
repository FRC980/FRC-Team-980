#include <Encoder.h>

#include "VelocityEncoder.h"

VelocityEncoder::VelocityEncoder(Encoder * pEnc)
    : m_pEnc(pEnc)
{
}

double VelocityEncoder::PIDGet()
{
    return this->m_pEnc->GetRate();
}
