#include <WPILib.h>

#include "SmartDrive.h"

SmartDrive::SmartDrive(SpeedController* psc,
                       Encoder* pEncDrive,
                       Encoder* pEncFollow)
    : m_psc(psc)
    , m_pEncDrive(pEncDrive)
    , m_pEncFollow(pEncFollow)
{
}

void SmartDrive::Set(float speed)
{
    m_psc->Set(speed);
}

float SmartDrive::Get()
{
    return m_psc->Get();
}
