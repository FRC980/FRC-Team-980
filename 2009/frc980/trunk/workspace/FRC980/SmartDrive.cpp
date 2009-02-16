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
    , m_kfPeriod(period)
    , m_dCmdSpeed(0)
    , m_bEnabled(false)
    , m_psc(psc)
    , m_pEncDrive(pEncDrive)
    , m_pEncFollow(pEncFollow)
    , m_controlLoop(new Notifier(SmartDrive::CallCalculate, this))
    , m_dPrevMotorVel(0)
    , m_dPrevRobotVel(0)
    , m_dVelInt(0)
    , m_dCorInt(0)
    , m_dAclInt(0)
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

    // This code implements 3 linked PID loops (technically PI, as there
    // is no D term).  The VELOCITY loop tries to make the motor run at
    // the commanded speed.  The ACCELERATION loop tries to maximize
    // acceleration.  The SLIPPAGE loop (aka "correction" loop) tries to
    // minimize slippage, (as measured by the difference between the motor
    // speed and follow wheel speed).


    // velocities should be in range of -1 to 1
    double dMotorVel = m_pEncDrive->GetRate(); // to do: scale
    double dRobotVel = m_pEncFollow->GetRate(); // to do: scale

    double dMotorAcl = (dMotorVel - m_dPrevMotorVel) / m_kfPeriod;
//    double dRobotAcl = (dRobotVel - m_dPrevRobotVel) / m_kfPeriod;

    double dVelDelta = m_dCmdSpeed - dMotorVel;

    m_dVelInt += dVelDelta * m_kVelI;
    m_dVelInt = limit(m_dVelInt, -1.0, 1.0);

    double dVelError = m_kVelP * dVelDelta + m_dVelInt;
    dVelError = limit(dVelError, -1.0, 1.0);

    double dSlippage = dMotorVel - dRobotVel;

    m_dCorInt += dSlippage * m_kCorI;
    m_dCorInt = limit(m_dCorInt, -1.25, 1.25);

    double dCor_out = dSlippage * m_kCorP + m_dCorInt;
    double dAclCmd = dVelError - dCor_out;

    double dAclError = dAclCmd - dMotorAcl;

    m_dAclInt += dAclError * m_kAclI;
    m_dAclInt = limit(m_dAclInt, -1.0, 1.0);

    double dMotorCmd = dAclError * m_kAclP + m_dAclInt;
    m_psc->Set(dMotorCmd);


    m_dPrevMotorVel = dMotorVel;
    m_dPrevRobotVel = dRobotVel;
}
