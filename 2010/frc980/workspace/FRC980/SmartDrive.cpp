#include <WPILib.h>
#include <Timer.h>
#include <math.h>

#include "SmartDrive.h"
#include "Robot980.h"
#include "main.h"
#include "utils.h"

extern bool DEBUG_bOK0;
extern bool DEBUG_bOK1;

//==============================================================================
//==============================================================================
SmartDrive::SmartDrive(uint8_t ID,
                       double kvp, double kvi,
                       double kcp, double kci,
                       double kap, double kai,
                       bool bUseAcl,
                       SpeedController* psc,
                       Encoder* pEncDrive,
                       Encoder* pEncFollow,
                       float period)
   : m_psc(psc)
   , m_pEncDrive(pEncDrive)
   , m_pEncFollow(pEncFollow)
   , m_pTimer(NULL)
   , m_controlLoop(new Notifier(SmartDrive::CallCalculate, this))   
   , m_ku8ID(ID)
   , m_kVelP(kvp)
   , m_kVelI(kvi)
   , m_kCorP(kcp)
   , m_kCorI(kci)
   , m_kAclP(bUseAcl ? kap : 1.0)
   , m_kAclI(bUseAcl ? kai : 0.0)
   , m_bUseAcl(bUseAcl)
   , m_bUseSlip(true)
   , m_kfPeriod(period)
   , m_dCmdSpeed(0)
   , m_bEnabled(false)
   , m_dPrevMotorVel(0)
   , m_dPrevRobotVel(0)
   , m_dVelInt(0)
   , m_dCorInt(0)
   , m_dAclInt(0)
   , m_dPrevMotorCount(0)
   , m_dPrevRobotCount(0)
{
   m_pTimer = new Timer;
   if (m_pTimer)
      m_pTimer->Start();

   m_controlLoop->StartPeriodic(period);
}

//==============================================================================
SmartDrive::~SmartDrive(void)
{
   delete m_controlLoop;
}

//==============================================================================
//==============================================================================
void SmartDrive::CallCalculate(void *pvsd)
{
   SmartDrive *psd = static_cast<SmartDrive*>(pvsd);
   psd->Calculate();
}

//==============================================================================
void SmartDrive::Calculate(void)
{
   //--- Set up utilities to use
   utils u;
   
   double dMotorCount = m_pEncDrive->GetDistance();
   double dRobotCount = m_pEncFollow->GetDistance();

   //--- velocities should be in range of -1 to 1
   // double dMotorVel = m_pEncDrive->GetRate(); // to do: scale
   // double dRobotVel = m_pEncFollow->GetRate(); // to do: scale
   double dMotorVel=(dMotorCount-m_dPrevMotorCount) / m_kfPeriod / TOP_SPEED;
   double dRobotVel=(dRobotCount-m_dPrevRobotCount) / m_kfPeriod / TOP_SPEED;

   double dTime = m_pTimer ? m_pTimer->Get() : 1;
   if (m_pTimer)
      m_pTimer->Reset();

#if 0
   if (!Main::getInstance().IsDisabled())
   {
      Dashboard &d = DriverStation::GetInstance()->GetDashboardPacker();

      if (!m_ku8ID ? (DEBUG_bOK0 && DEBUG_bOK1) : (DEBUG_bOK1 && !DEBUG_bOK0))
      {
         d.Printf("%d: T: %6.4f  Vel: %7.4f  Acl: %7.4f  Slp: %7.4f\n",
                  m_ku8ID, dTime, dMotorVel, dMotorVel - m_dPrevMotorVel,
                  dMotorVel - dRobotVel);

         if (!m_ku8ID)
            DEBUG_bOK0 = false;
         else
            DEBUG_bOK1 = false;
      }
   }
#endif // 0

   if (m_bEnabled)
   {
      double dMotorAcl =
          m_bUseAcl ? (dMotorVel - m_dPrevMotorVel) / m_kfPeriod : 0;
      // double dRobotAcl = (dRobotVel - m_dPrevRobotVel) / m_kfPeriod;
      
      double dVelDelta = m_dCmdSpeed - dMotorVel;
      
      m_dVelInt += dVelDelta * m_kVelI;
      m_dVelInt = u.limit(m_dVelInt, -1.0, 1.0);
      
      double dVelError = m_kVelP * dVelDelta + m_dVelInt;
      dVelError = u.limit(dVelError, -1.0, 1.0);
      
      double dSlippage = m_bUseSlip ? (dMotorVel - dRobotVel) : 0;
      
      //--- REPLACED ABS FROM UTILS.H WITH abs() FROM MATH.H
      if (abs(dSlippage) <= .01){
         dSlippage = 0;
      }
      
      m_dCorInt += dSlippage * m_kCorI;
      // m_dCorInt = u.limit(m_dCorInt, -1.25, 1.25);
      m_dCorInt = u.limit(m_dCorInt, -0.9, 0.9);
      
      double dCor_out = dSlippage * m_kCorP + m_dCorInt;
      double dAclCmd = dVelError - dCor_out;
      double dAclError = dAclCmd - dMotorAcl;
      
      m_dAclInt += dAclError * m_kAclI;
      m_dAclInt = u.limit(m_dAclInt, -1.0, 1.0);
      
      double dMotorCmd = dAclError * m_kAclP + m_dAclInt;
      
      if (m_bUseSlip){
          m_psc->Set(dMotorCmd);
      }
      else
      {
         double d = m_psc->Get();
         m_psc->Set(u.limit(dMotorCmd, d - dTime, d + dTime));
      }
   }

   m_dPrevMotorCount = dMotorCount;
   m_dPrevRobotCount = dRobotCount;

   m_dPrevMotorVel = dMotorVel;
   m_dPrevRobotVel = dRobotVel;
}

//==============================================================================
//==============================================================================
void SmartDrive::Set(float speed)
{
   //--- Ensure that the speed is limited and then set the speed
   utils u;
   this->m_dCmdSpeed = u.limit(speed);
}

//==============================================================================
float SmartDrive::Get(void)
{
   //--- Return the current command speed
   return this->m_dCmdSpeed;
}

//==============================================================================
//==============================================================================
void SmartDrive::Enable(bool bUseSlip)
{
   this->m_bEnabled = true;
   this->m_bUseSlip = bUseSlip;
   if (!this->m_bUseSlip)
      this->m_dCorInt = 0;
}

//==============================================================================
void SmartDrive::Disable(void)
{
   this->m_bEnabled = false;
   // m_psc->Set(0);

   this->m_dVelInt = 0;
   this->m_dCorInt = 0;
   this->m_dAclInt = 0;
}

//==============================================================================
void SmartDrive::Reset(void)
{
   //--- Disable the speed controller
   this->Disable();
}
