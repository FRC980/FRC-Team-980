#include <WPILib.h>

#include "SmartDrive.h"

double limit(double val, double min = -1, double max = 1)
{
    if (val > max)
        return max;

    if (val < min)
        return min;

    return val;
}

SmartDrive::SmartDrive(double kvp, double kvi, double kvd,
                       double kcp, double kci, double kcd,
                       double kap, double kai, double kad,
                       SpeedController* psc,
                       Encoder* pEncDrive,
                       Encoder* pEncFollow,
                       float period)
    : m_kVelP(kvp)
    , m_kVelI(kvi)
    , m_kVelD(kvd)
    , m_kCorP(kcp)
    , m_kCorI(kci)
    , m_kCorD(kvd)
    , m_kAclP(kap)
    , m_kAclI(kai)
    , m_kAclD(kad)
    , m_dCmdSpeed(0)
    , m_bEnabled(false)
    , m_psc(psc)
    , m_pEncDrive(pEncDrive)
    , m_pEncFollow(pEncFollow)
    , m_controlLoop(new Notifier(SmartDrive::CallCalculate, this))
{
    m_controlLoop->StartPeriodic(period);
}

SmartDrive::~SmartDrive()
{
    delete m_controlLoop;
}

/**
 * Call the Calculate method as a non-static method. This avoids having to
 * prepend all local variables in that method with the class pointer. This
 * way the "this" pointer will be set up and class variables can be called
 * more easily.  This method is static and called by the Notifier class.
 * @param controller the address of the PID controller object to use in
 * the background loop
 */
void SmartDrive::CallCalculate(void *pvsd)
{
    SmartDrive *psd = static_cast<SmartDrive*>(pvsd);
    psd->Calculate();
}


void SmartDrive::Enable()
{
    m_bEnabled = true;
}

void SmartDrive::Disable()
{
    m_bEnabled = false;
    m_psc->Set(0);
}

void SmartDrive::Reset()
{
    Disable();
}

void SmartDrive::Calculate()
{
    if (!m_bEnabled)
        return;

    double dMotorVel = m_pEncDrive->GetRate();
    double dRobotVel = m_pEncFollow->GetRate();
}
