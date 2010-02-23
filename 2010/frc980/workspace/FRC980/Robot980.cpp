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
   //, m_pscWinch(new CANJaguar(CAN_WINCH))//, CANJaguar::kSpeed))
   
   //--- Victors
   , m_pscArm1_win(new Victor(DSC_SLOT, CHAN_PWM_ARM1))
   , m_pscArm2_win(new Victor(DSC_SLOT, CHAN_PWM_ARM2))
   , m_pscFire_win(new Victor(DSC_SLOT, CHAN_PWM_FIRE))
   
   //--- Sensors
   //, m_pGyro(new Gyro(SLOT_GYRO, CHAN_GYRO))
   , m_pscArm_switch(new DigitalInput(DSC_SLOT, CHAN_LIMIT_ARM))
   , m_pscFire_switch(new DigitalInput(DSC_SLOT, CHAN_LIMIT_FIRE))
   , m_pscWinch_switch(new DigitalInput(DSC_SLOT, CHAN_LIMIT_WINCH))

   //--- Timers
   , m_pTimerDrive(new Timer)
   , m_pTimerFire(new Timer)
{	
   //--- Set the PID Values for each Jag in Speed or Current Mode
   //double kp = 0.35;
   //double ki = 0.003;
   //double kd = 0.001;
   //this->m_pscLeft_cim->SetPID( kp,  ki,  kd);
   //this->m_pscLeft_fp->SetPID( kp,  ki,  kd);
   //this->m_pscRight_cim->SetPID( kp,  ki,  kd);
   //this->m_pscRight_fp->SetPID( kp,  ki,  kd);
   //this->m_pscRoller_cim->SetPID( kp,  ki,  kd);
   //this->m_pscWinch->SetPID( kp,  ki,  kd);
   
   //--- Encoder setup for Left CIM
   //this->m_pscLeft_cim->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
   //this->m_pscLeft_cim->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
   //this->m_pscLeft_cim->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

   //--- Encoder setup for Right CIM
   //this->m_pscRight_cim->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
   //this->m_pscRight_cim->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
   //this->m_pscRight_cim->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

   //--- Encoder setup for Roller
   //this->m_pscRoller_cim->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
   //this->m_pscRoller_cim->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
   //this->m_pscRoller_cim->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);

   //--- Define Drive Timer
   this->m_pTimerDrive->Reset();
   this->m_pTimerDrive->Start();

   //--- Define Fire Timer
   this->m_pTimerFire->Reset();
   this->m_pTimerFire->Start();

   //--- Tell SensorBase about us
   AddToSingletonList();
   
   //--- Set up winch
   this->unwindWinch = false;
   this->countWinch = 0;
}

//==============================================================================
Robot980::~Robot980()
{
   //--- Speed controllers
   delete this->m_pscLeft_cim;
   delete this->m_pscLeft_fp;
   delete this->m_pscRight_cim;
   delete this->m_pscRight_fp;
   
   delete this->m_pscRoller_cim;
   //delete this->m_pscWinch;
   
   delete this->m_pscArm1_win;
   delete this->m_pscArm2_win;
   delete this->m_pscFire_win;

   //--- Sensors
   //delete this->m_pGyro;
   delete this->m_pscArm_switch;
   delete this->m_pscFire_switch;
   
   //--- Timers
   delete this->m_pTimerDrive;
   delete this->m_pTimerFire;
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
   this->m_pTimerDrive->Reset();

   //--- Set the speed of the left motors
   this->m_pscLeft_cim->Set(left);
   this->m_pscLeft_fp->Set(left);
   
   //--- Set the speed of the right motors
   this->m_pscRight_cim->Set(right);
   this->m_pscRight_fp->Set(right);
   
   //--- Set the speed of the roller motor
   this->m_pscRoller_cim->Set(roller);
}

//==============================================================================
void Robot980::Drive(float left, float right)
{
   //--- Reset the Timer Drive
   this->m_pTimerDrive->Reset();

   //--- Set the speed of the left motors
   this->m_pscLeft_cim->Set(left*DRIVE_REVERSE);
   this->m_pscLeft_fp->Set(left*DRIVE_REVERSE);
   
   //--- Set the speed of the right motors
   this->m_pscRight_cim->Set(right); 
   this->m_pscRight_fp->Set(right);  
   
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
      this->m_pscRoller_cim->Set(0.0);
   }
   else if(fForwardSpeed == 0){
	  this->m_pscRoller_cim->Set(-0.5); // Speed is limited by set command
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
      this->m_pscRoller_cim->Set(speed); // Speed is limited by set command
   }
   
}

//==============================================================================
//==============================================================================
bool Robot980::KickerArmed(void)
{
   if(this->m_pscArm_switch->Get())
      return true;
   return false;
}

//==============================================================================
void Robot980::ArmKicker(void)
{
   if(!this->KickerArmed() && !this->unwindWinch){
	  //--- Wind up the winch to arm the kicker
	  this->m_pscArm1_win->Set(1.0);
	  this->m_pscArm2_win->Set(1.0);
	  
	  //--- Disable the unwind function for the winch
	  this->unwindWinch = false;
   }
}

//==============================================================================
void Robot980::StopArmWinch(void)
{
   //--- Stop the kicker winch
   if(this->KickerArmed() && !this->unwindWinch){
	  //--- Stop the arming of the kicker with the winch
	  this->m_pscArm1_win->Set(0.0);
	  this->m_pscArm2_win->Set(0.0);
	  
	  //--- Enable the unwind function for the winch
	  this->unwindWinch = true;
   }
}

//==============================================================================
void Robot980::UnwindWinch(void)
{
   if(this->KickerArmed() && this->unwindWinch){
	  
	  //--- If the count is less than 2
      if(this->countWinch < 2){
	  
	     //--- Unwind the winch for the kciker
	     this->m_pscArm1_win->Set(-1.0);
	     this->m_pscArm2_win->Set(-1.0);
      
	     //--- If the winch switch is hit then add one to the count
	     //    The count must equal 2 to stop unwinding
	     if(this->m_pscWinch_switch->Get()){
            this->countWinch += 1;
	     }
      }
      //--- If the count is greater than 2
      else {
	     //--- Disable the unwind function for the winch
	     this->unwindWinch = false;
	     this->countWinch = 0;
      }
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
   if(this->KickerArmed() && 
      !this->unwindWinch &&
      this->m_pTimerFire->Get() > KICKER_RESET_PERIOD){
      
	  //--- Reset the firing timer
	  this->m_pTimerFire->Reset();
	   
	  //--- Run the Kick motor to release the kicker
	  this->m_pscFire_win->Set(1.0);
   }
}

//==============================================================================
void Robot980::StopKickerCam(void)
{
	//--- If the fire switch was hit then stop the fire cam from moving
	if(this->m_pscFire_switch->Get()){
		this->m_pscFire_win->Set(0.0);
	}
}

//==============================================================================
//void Robot980::Lift(void)
//{
//   
//}

//==============================================================================
//==============================================================================
//float Robot980::GetAngle(void)
//{
//   return 1.0;
//}

