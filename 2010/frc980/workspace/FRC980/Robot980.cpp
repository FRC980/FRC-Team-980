#include <WPILib.h>
#include <NetworkCommunication/FRCComm.h>
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
   : m_pscLeft_cim1(new CANJaguar(CAN_LEFT_CIM1))//, CANJaguar::kSpeed))
   , m_pscLeft_cim2(new CANJaguar(CAN_LEFT_CIM2))//, CANJaguar::kSpeed))
   , m_pscRight_cim1(new CANJaguar(CAN_RIGHT_CIM1))//, CANJaguar::kSpeed))
   , m_pscRight_cim2(new CANJaguar(CAN_RIGHT_CIM2))//, CANJaguar::kSpeed))
   
   // roller and lift motors
   , m_pscRoller_fp(new CANJaguar(CAN_ROLLER_FP))//, CANJaguar::kSpeed))
   //, m_pscLift(new CANJaguar(CAN_LIFT))//, CANJaguar::kSpeed))
   
   //--- Victors
   , m_pscArm1_win(new Victor(DSC_SLOT, CHAN_PWM_ARM))
   , m_pscArm2_win(new Victor(DSC_SLOT, CHAN_PWM_ARM))
   , m_pscFire_win(new Victor(DSC_SLOT, CHAN_PWM_FIRE))
   
   //--- Sensors
   //, m_pGyro(new Gyro(SLOT_GYRO, CHAN_GYRO))
   , m_pdiArm_switch(new DigitalInput(DSC_SLOT, CHAN_LIMIT_ARMED))
   , m_pdiFire_switch(new DigitalInput(DSC_SLOT, CHAN_LIMIT_FIRE_READY))
   , m_pdiWinch_switch(new DigitalInput(DSC_SLOT, CHAN_LIMIT_WINCH_COUNTER))

   //--- Timers
   , m_pTimerDrive(new Timer)
   , m_pTimerFire(new Timer)
{	
   //--- Set the PID Values for each Jag in Speed or Current Mode
   //double kp = 0.35;
   //double ki = 0.003;
   //double kd = 0.001;
   //this->m_pscLeft_cim1->SetPID( kp,  ki,  kd);
   //this->m_pscLeft_cim2->SetPID( kp,  ki,  kd);
   //this->m_pscRight_cim1->SetPID( kp,  ki,  kd);
   //this->m_pscRight_cim2->SetPID( kp,  ki,  kd);
   //this->m_pscRoller_cim->SetPID( kp,  ki,  kd);
   
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
   this->m_bUnwindWinch = false;
   this->m_iCountWinch = 0;
   this->m_bOldWinchState = true;
}

//==============================================================================
Robot980::~Robot980()
{
   //--- Speed controllers
   delete this->m_pscLeft_cim1;
   delete this->m_pscLeft_cim2;
   delete this->m_pscRight_cim1;
   delete this->m_pscRight_cim2;
   
   delete this->m_pscRoller_fp;
   //delete this->m_pscLift;
   
   delete this->m_pscArm1_win;
   delete this->m_pscArm2_win;
   delete this->m_pscFire_win;

   //--- Sensors
   //delete this->m_pGyro;
   delete this->m_pdiArm_switch;
   delete this->m_pdiFire_switch;
   
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
   //--- Get the analog value corresponsind to the autonomous mode to choose
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
   
   //--- Set up utilities to use
   utils u;
   
   //--- Set the speed of the left motors
   this->m_pscLeft_cim1->Set(u.limit(left*DRIVE_REVERSE));
   this->m_pscLeft_cim2->Set(u.limit(left*DRIVE_REVERSE));
   
   //--- Set the speed of the right motors
   this->m_pscRight_cim1->Set(u.limit(right));
   this->m_pscRight_cim2->Set(u.limit(right));
   
   //--- Set the speed of the roller motor
   //this->m_pscRoller_cim->Set(u.limit(roller));
}

//==============================================================================
void Robot980::Drive(float left, float right)
{  
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
   
   /* IMPORTANT!
    * 
    * IF THE ROLLER IS GOING OPPOSITE THAN DESIRED THE 
    * fForwardSpeed VARIABLE NEEDS TO BE REVERSED, BUT
    * NOT THE ROLLER DRIVE SPEED.
    * 
    */
   
   //--- Set when going forward
   if (fForwardSpeed > 0){
      this->Drive(left, right, 0.0);
   }
   //--- Set a constant speed if not moving forward
   else if(fForwardSpeed == 0){
       this->Drive(left, right, -0.5);
   }
   //--- Set a variable roller speed when moving backwards
   else
   {
      // Roller runs from a CIM through an 11:3 gearbox.  The Roller is
      // a 3" dia; the ball is 9" dia, and the wheels are 6" dia.  This
      // means RPM of ball is 2/3 RPM of wheels, and roller is 3x RPM of
      // ball, making roller 2x RPM of wheels.  We then add an extra
      // 10%.
      fForwardSpeed *= 2 * 1.1 * (ROLLER_GEARBOX) / (GEARBOX_RATIO);
      this->Drive(left, right, fForwardSpeed);
   }
}

//==============================================================================
//==============================================================================
bool Robot980::KickerRetracted(void)
{
   if(this->m_pdiArm_switch->Get()){
      return true;
   }
   return false;
}

//==============================================================================
bool Robot980::KickerArmed(void)
{
   if(this->KickerRetracted() && 
      !this->m_bUnwindWinch &&
      this->m_pTimerFire->Get() > KICKER_RESET_PERIOD){
        return true;
   }
   return false;
}

//==============================================================================
bool Robot980::KickerFired(void)
{
   return !this->KickerRetracted();
}

//==============================================================================
void Robot980::ArmKicker(void)
{
    char error[256];
    sprintf(error, "ArmKicker:");
    sprintf(error, "%d %d %d \n", this->m_pdiArm_switch->Get(), 
            this->m_pdiFire_switch->Get(), this->m_pdiWinch_switch->Get()  );
    setUserDsLcdData(error,strlen(error),100);
        //setErrorData?
   //--- Arm the kicker
   if(!this->KickerRetracted() && !this->m_bUnwindWinch){
	  //--- Wind up the winch to arm the kicker
	  this->m_pscArm1_win->Set(0.25);
	  this->m_pscArm2_win->Set(0.25);
	  
	  //--- Disable the unwind function for the winch
	  this->m_bUnwindWinch = false;
   }
   //--- Stop the kicker winch
   else if(this->KickerRetracted() && !this->m_bUnwindWinch){
	  //--- Stop the arming of the kicker with the winch
	  this->m_pscArm1_win->Set(0.0);
	  this->m_pscArm2_win->Set(0.0);
	  
	  //--- Enable the unwind function for the winch
	  this->m_bUnwindWinch = true;
	  this->m_iCountWinch = 0;
   }
   //--- Unwind the kicker winch
   else if(this->KickerRetracted() && this->m_bUnwindWinch){
	  
	  //--- If the state is less than 2 continue to unwind
      if(this->m_iCountWinch < 2){
	  
	     //--- Unwind the winch for the kicker
	     this->m_pscArm1_win->Set(-0.25);
	     this->m_pscArm2_win->Set(-0.25);
      
	     //--- Switch hit: increment the state
	     if(this->m_pdiWinch_switch->Get() && !m_bOldWinchState){
	        this->m_bOldWinchState = true;
	     }
	     //--- Switch released: old is now "off"
	     else if(!this->m_pdiWinch_switch->Get() && m_bOldWinchState){
	        this->m_bOldWinchState = false;
            this->m_iCountWinch += 1;
         }
      }
      //--- If the count is 2 or greater, stop unwinding
      else {
	     //--- Disable the unwind function for the winch
	     this->m_bUnwindWinch = false;
      }
   }
}

//==============================================================================
void Robot980::FireKicker(void)
{
   if(this->KickerArmed()){
      
	  //--- Reset the firing timer
	  this->m_pTimerFire->Reset();
	   
	  //--- Run the Kick motor to release the kicker
	  this->m_pscFire_win->Set(0.25);
   }
}

//==============================================================================
void Robot980::StopKickerCam(void)
{
	//--- If the fire switch was hit then stop the fire cam from moving
	if(this->m_pdiFire_switch->Get()){
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

