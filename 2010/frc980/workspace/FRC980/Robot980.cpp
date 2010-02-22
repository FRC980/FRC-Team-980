#include <WPILib.h>
//#include <PCVideoServer.h>
#include <Timer.h>
#include <math.h>
#include <stdbool.h>

//#include "ReversableCANJaguar.h"
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
   //--- Jaguars (Encoders attached directly to Jaguars)
   //    NOTE: CANNOT RUN IN kSpeed MODE UNLESS ENCODERS ATTACHED
   // left and right drive motors
   : m_pscLeft_cim(new CANJaguar(CAN_LEFT_CIM))//, CANJaguar::kSpeed))
   , m_pscLeft_fp(new CANJaguar(CAN_LEFT_FP))//, CANJaguar::kSpeed))
   , m_pscRight_cim(new CANJaguar(CAN_RIGHT_CIM))//, CANJaguar::kSpeed))
   , m_pscRight_fp(new CANJaguar(CAN_RIGHT_FP))//, CANJaguar::kSpeed))
   
   // roller and winch motors
   , m_pscRoller_cim(new CANJaguar(CAN_ROLLER_CIM))//, CANJaguar::kSpeed))
   , m_pscWinch(new CANJaguar(CAN_WINCH))//, CANJaguar::kSpeed))
   
   //--- Victors
   , m_pscArm1(new Victor(DSC_SLOT, CHAN_PWM_ARM1))
   , m_pscArm2(new Victor(DSC_SLOT, CHAN_PWM_ARM2))
   , m_pscFire(new Victor(DSC_SLOT, CHAN_PWM_FIRE))
   
   //--- Sensors
   //, m_pGyro(new Gyro(SLOT_GYRO, CHAN_GYRO))

   //--- Timers
   , m_pTimerDrive(new Timer)
   , m_pTimerFire(new Timer)
{	
   //--- Set the PID Values for each Jag in Speed or Current Mode
   //double kp = 0.35;
   //double ki = 0.003;
   //double kd = 0.001;
   //m_pscLeft_cim->SetPID( kp,  ki,  kd);
   //m_pscLeft_fp->SetPID( kp,  ki,  kd);
   //m_pscRight_cim->SetPID( kp,  ki,  kd);
   //m_pscRight_fp->SetPID( kp,  ki,  kd);
   //m_pscRoller_cim->SetPID( kp,  ki,  kd);
   //m_pscWinch->SetPID( kp,  ki,  kd);
   
   //--- Encoder setup for Left CIM
   //m_pscLeft_cim->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
   //m_pscLeft_cim->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
   //m_pscLeft_cim->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

   //--- Encoder setup for Right CIM
   //m_pscRight_cim->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
   //m_pscRight_cim->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
   //m_pscRight_cim->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

   //--- Encoder setup for Roller
   //m_pscRoller_cim->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
   //m_pscRoller_cim->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
   //m_pscRoller_cim->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

   //--- Define Drive Timer
   m_pTimerDrive->Reset();
   m_pTimerDrive->Start();

   //--- Define Fire Timer
   m_pTimerFire->Reset();
   m_pTimerFire->Start();

   //--- Tell SensorBase about us
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
   
   delete m_pscRoller_cim;
   delete m_pscWinch;
   
   delete m_pscArm1;
   delete m_pscArm2;
   delete m_pscFire;

   //--- Sensors
   //delete m_pGyro;
   
   //--- Timers
   delete m_pTimerDrive;
   delete m_pTimerFire;
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
   m_pscLeft_fp->Set(left);
   
   //--- Set the speed of the right motors
   m_pscRight_cim->Set(right);
   m_pscRight_fp->Set(right);
   
   //--- Set the speed of the roller motor
   m_pscRoller_cim->Set(roller);
}

//==============================================================================
void Robot980::Drive(float left, float right)
{
   //--- Reset the Timer Drive
   m_pTimerDrive->Reset();

   //--- Set the speed of the left motors
   m_pscLeft_cim->Set(left);
   m_pscLeft_fp->Set(left);
   
   //--- Set the speed of the right motors
   m_pscRight_cim->Set(right); //Set(right*DRIVE_REVERSE);
   m_pscRight_fp->Set(right);  //Set(right*DRIVE_REVERSE);
   
   //--- Set the speed of the roller motor based upon the forward/back speed
   //    The forward speed here represents a direct relation to the y-axis
   //    input of the command joystick.
   //
   //    Ex: fForwardSpeed = u.limit(((y-x) + (y+x))/2)
   //                      = u.limit((y-x+y+x)/2)
   //                      = u.limit((2*y)/2)
   //                      = u.limit(y)
   //
   utils u;
   float fForwardSpeed = u.limit((left + right) / 2);
   
   //--- Set when going forward
   if (fForwardSpeed > 0){
      m_pscRoller_cim->Set(0.0);
   }
   else if(fForwardSpeed == 0){
	  m_pscRoller_cim->Set(-0.5); // Speed is limited by set command
   }
   //--- Set when moving backwards
   else
   {
      // Roller runs from a CIM through an 11:3 gearbox.  The Roller is
      // a 3" dia; the ball is 9" dia, and the wheels are 6" dia.  This
      // means RPM of ball is 2/3 RPM of wheels, and roller is 3x RPM of
      // ball, making roller 2x RPM of wheels.  We then add an extra
      // 10%.
      float speed = fForwardSpeed;
      speed *= 2 * 1.1 * (ROLLER_GEARBOX) / (GEARBOX_RATIO);
      m_pscRoller_cim->Set(speed); // Speed is limited by set command
   }
   
}

//==============================================================================
void Robot980::Lift(void)
{
   
}

//==============================================================================
//==============================================================================
bool Robot980::KickerArmed(void)
{
   // IF THE SWITCH IS TRIPPED FOR THE KICKING CAM MOTOR
   // AND THE SWITCH IS TRIPPED BECAUSE THE KICKING MECHANISM IS ALL THE WAY BACK
   // THEN RETURN TRUE, ELSE FALSE
   return false;
}

//==============================================================================
void Robot980::ArmKicker(void)
{
   if(!this->KickerArmed()){
	   // RUN MOTOR UNTIL SWITCH IS TRIPPED AND UNWIND FOR TWO SECONDS
   }
}

//==============================================================================
bool Robot980::KickerFired(void)
{
   return !this->KickerArmed();
}

//==============================================================================
void Robot980::FireKicker(void)
{
   if(this->KickerArmed() && m_pTimerFire->Get() > KICKER_RESET_PERIOD){
	   //--- Reset the firing timer
	   m_pTimerFire->Reset();
	   
	   //--- Run the Kick motor to release the kicker
   }
}

//==============================================================================
//==============================================================================
//float Robot980::GetAngle(void)
//{
//   return 1.0;
//}

