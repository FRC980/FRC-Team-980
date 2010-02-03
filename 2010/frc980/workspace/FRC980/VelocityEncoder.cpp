#include <Encoder.h>

#include "VelocityEncoder.h"

VelocityEncoder::VelocityEncoder(Encoder* pEnc)
    : m_pEnc(pEnc)
{
}

double VelocityEncoder::PIDGet()
{
    return m_pEnc->GetRate();
}

