#include <WPILib.h>
#include <PCVideoServer.h>
#include <Timer.h>
#include <math.h>
#include <stdbool.h>

#include "ReversableCANJaguar.h"
#include "CANJaguar.h"
#include "Robot980.h"
#include "numbers.h"
#include "utils.h"

#define ENC_SCALE   CounterBase::k1X   /*!< \def ENC_SCALE The encoder scaling */

//==============================================================================
static Robot980* g_pInstance = NULL;

//==============================================================================
//==============================================================================
Robot980::Robot980()
   //--- Jaguars
   // left and right drive motors
   : m_pscLeft_cim(new ReversableCANJaguar(CAN_LEFT_CIM, true))
   , m_pscLeft_fp (new ReversableCANJaguar(CAN_LEFT_FP, true))
   , m_pscRight_cim(new CANJaguar(CAN_RIGHT_CIM))
   , m_pscRight_fp (new CANJaguar(CAN_RIGHT_FP))
   
   // roller and winch motors
   , m_pscRoller(new CANJaguar(CAN_ROLLER))
   , m_pscWinch(new CANJaguar(CAN_WINCH))
   
   //--- Victors
   , m_pscArm1(new Victor(DSC_SLOT, CHAN_PWM_ARM1))
   , m_pscArm2(new Victor(DSC_SLOT, CHAN_PWM_ARM2))
   , m_pscFire(new Victor(DSC_SLOT, CHAN_PWM_FIRE))

   //--- Encoders on the drive wheels
   , m_pEncDrvLeft(new Encoder(DSC_SLOT,
                               CHAN_ENC_DRV_LEFT_A,
                               DSC_SLOT,
                               CHAN_ENC_DRV_LEFT_B,
                               false, ENC_SCALE))
   , m_pEncDrvRight(new Encoder(DSC_SLOT,
                                CHAN_ENC_DRV_RIGHT_A,
                                DSC_SLOT,
                                CHAN_ENC_DRV_RIGHT_B,
                                false, ENC_SCALE))
   
   //--- Sensors
   , m_pGyro(new Gyro(SLOT_GYRO, CHAN_GYRO))

   //--- Timers
   , m_pTimerDrive(new Timer)
   , m_pTimerFire(new Timer)
   
   //--- Camera
   , m_pSrvPan(new Servo(DSC_SLOT, CAMERA_CHAN_PAN))
   , m_pSrvTilt(new Servo(DSC_SLOT, CAMERA_CHAN_TILT))
   , m_pVideoServer(NULL)
{
   // pi * diameter * gear ratio / encoder ticks / in/ft
   m_pEncDrvLeft->SetDistancePerPulse(M_PI * 6 * GEAR_RATIO / 250 / 12);
   m_pEncDrvRight->SetDistancePerPulse(M_PI * 6 * GEAR_RATIO / 250 / 12);

   m_pEncDrvLeft->Start();
   m_pEncDrvRight->Start();

   m_pTimerDrive->Reset();
   m_pTimerDrive->Start();

   m_pTimerFire->Reset();
   m_pTimerFire->Start();

   //--- start the CameraTask
   m_pVideoServer = new PCVideoServer;

   // tell SensorBase about us
   AddToSingletonList();
}

//==============================================================================
Robot980::~Robot980()
{
   //--- Speed controllers
   delete m_pscLeft_cim;
   delete m_pscLeft_fp;
   delete m_pscRight_cim;
   delete m_pscRight_fp;
   
   delete m_pscRoller;
   delete m_pscWinch;
   
   delete m_pscArm1;
   delete m_pscArm2;
   delete m_pscFire;
   
   //--- Encoders
   delete m_pEncDrvLeft;
   delete m_pEncDrvRight;

   //--- Sensors
   delete m_pGyro;
   
   //--- Timers
   delete m_pTimerDrive;
   delete m_pTimerFire;

   //--- Camera
   delete m_pSrvPan;
   delete m_pSrvTilt;
   delete m_pVideoServer;
}

//==============================================================================
Robot980* Robot980::GetInstance()
{
   if (!g_pInstance)
   {
      g_pInstance = new Robot980();
   }
   return g_pInstance;
}

//==============================================================================
//==============================================================================
int Robot980::GetAutonMode()
{
   AnalogModule *pAM = AnalogModule::GetInstance(SLOT_AUTO_MODE);
   int i = pAM->GetValue(CHAN_AUTO_MODE); // returns 10-bit number

   // REVIEW: These are samples; exact values and quantities depend on
   // what's actually built and need to be measured and tested.
   if (i > 900)
      return 6;
   if (i > 750)
      return 5;
   if (i > 600)
      return 4;
   if (i > 450)
      return 3;
   if (i > 300)
      return 2;
   if (i > 150)
      return 1;

   return 0;
}

//==============================================================================
//==============================================================================
void Robot980::Drive(float left, float right, float roller)
{
   //--- Reset the Timer Drive
   m_pTimerDrive->Reset();

   //--- Set the speed of the left motors
   m_pscLeft_cim->Set(left);
   //m_pscLeft_fp->Set(left);
   
   //--- Set the speed of the right motors
   m_pscRight_cim->Set(right);
   //m_pscRight_fp->Set(right);
   
   //--- Set the speed of the roller motor
   m_pscRoller->Set(roller);
}

//==============================================================================
void Robot980::Drive(float left, float right)
{
   //--- Get access to the utilities
   utils u;
   
   //--- Reset the Timer Drive
   m_pTimerDrive->Reset();

   //--- Set the speed of the left motors
   m_pscLeft_cim->Set(left);
   //m_pscLeft_fp->Set(left);
   
   //--- Set the speed of the right motors
   m_pscRight_cim->Set(right);
   //m_pscRight_fp->Set(right);
   
   //--- Set the speed of the roller motor based upon the left and right speeds
   //--- Set when going forward
   if (left + right > 0){
      m_pscRoller->Set(0);
   }
   //--- Set when moving backwards
   else
   {
      // Roller runs from a CIM through an 11:3 gearbox.  The Roller is
      // a 3" dia; the ball is 9" dia, and the wheels are 6" dia.  This
      // means RPM of ball is 2/3 RPM of wheels, and roller is 3x RPM of
      // ball, making roller 2x RPM of wheels.  We then add an extra
      // 10%.
      double speed = (left + right) / 2;
      speed *= 2 * 1.1 * (ROLLER_GEARBOX) / (GEARBOX_RATIO);
      m_pscRoller->Set(u.limit(speed));
   }
}

//==============================================================================
void Robot980::Lift(float speed)
{
   
}

//==============================================================================
//==============================================================================
bool Robot980::KickerArmed(void)
{
   if (! CanKick())
      return false;

   return false;
}

//==============================================================================
void Robot980::ArmKicker(void)
{
   
}

//==============================================================================
bool Robot980::KickerFired(void)
{

   return false;
}

//==============================================================================
void Robot980::FireKicker(void)
{
   
}

//==============================================================================
//==============================================================================
float Robot980::GetAngle(void)
{
   return 1.0;
}

//==============================================================================
//==============================================================================
bool TargetAvailable(void)
{
   return false;
}
