#include <WPILib.h>
#include <NetworkCommunication/FRCComm.h>
#include <Timer.h>
//#include <Vision/PCVideoServer.h>
#include <math.h>
#include <stdbool.h>

//#include "ReversableCANJaguar.h"
#include "CANJaguar.h"
#include "Robot980.h"
#include "numbers.h"
#include "utils.h"

#define ENC_SCALE   CounterBase::k1X /*!< \def ENC_SCALE The encoder scaling */

//==========================================================================
static Robot980 *g_pInstance = NULL;

//==========================================================================
//==========================================================================
Robot980::Robot980()
    //--- Jaguars (Encoders attached directly to Jaguars)
    // NOTE: CANNOT RUN IN kSpeed MODE UNLESS ENCODERS ATTACHED
    // left and right drive motors
    : m_pscLeft1(new CANJaguar(CAN_LEFT_DRIVE1))   //, CANJaguar::kSpeed))
    , m_pscLeft2(new CANJaguar(CAN_LEFT_DRIVE2))   //, CANJaguar::kSpeed))
    , m_pscRight1(new CANJaguar(CAN_RIGHT_DRIVE1)) //, CANJaguar::kSpeed))
    , m_pscRight2(new CANJaguar(CAN_RIGHT_DRIVE2)) //, CANJaguar::kSpeed))

    , m_pscMiniDeploy(new CANJaguar(CAN_MINIDEPLOY))
    , m_pscClaw(new CANJaguar(CAN_ARM_CLAW))

      //--- Victors
    , m_pscShoulder(new Victor(DSC_SLOT, CHAN_PWM_SHOULDER))
      //--- Sensors
    , m_pGyro(new Gyro(SLOT_GYRO, CHAN_GYRO))
    , m_pdiLineLeft(new DigitalInput(DSC_SLOT, CHAN_LINE_LEFT))
    , m_pdiLineCenter(new DigitalInput(DSC_SLOT, CHAN_LINE_CENTER))
    , m_pdiLineRight(new DigitalInput(DSC_SLOT, CHAN_LINE_RIGHT))
      //--- Timers
    , m_pTimerDrive(new Timer)

      //--- State variables
      //--- Camera
    //, m_pVideoServer(NULL)
    //, m_pVideoServer(new PCVideoServer)
{
    //--- Set the PID Values for each Jag in Speed or Current Mode
    //double kp = 0.35;
    //double ki = 0.003;
    //double kd = 0.001;
    //m_pscLeft1->SetPID( kp,  ki,  kd);
    //m_pscLeft2->SetPID( kp,  ki,  kd);
    //m_pscRight1->SetPID( kp,  ki,  kd);
    //m_pscRight2->SetPID( kp,  ki,  kd);

    //--- Encoder setup for Left CIMs
    m_pscLeft1->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
    m_pscLeft1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscLeft1->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
    m_pscLeft1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
    
    m_pscLeft2->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
    m_pscLeft2->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscLeft2->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
    m_pscLeft2->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    //--- Encoder setup for Right CIMs
    m_pscRight1->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
    m_pscRight1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscRight1->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
    m_pscRight1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscRight2->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
    m_pscRight2->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscRight2->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
    m_pscRight2->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);


    //--- Define Drive Timer
    m_pTimerDrive->Reset();
    m_pTimerDrive->Start();

    //--- Tell SensorBase about us
    AddToSingletonList();

    // if (m_pVideoServer)
    // {
    //     AxisCamera::GetInstance().WriteRotation(AxisCamera::kRotation_180);
    //     AxisCamera::GetInstance().WriteResolution(AxisCamera::kResolution_320x240);
    //     AxisCamera::GetInstance().WriteMaxFPS(15);
    //     m_pVideoServer->Start();
    // }
}

//==========================================================================
Robot980::~Robot980()
{
    // if (m_pVideoServer)
    // {
    //     m_pVideoServer->Stop();
    //     delete m_pVideoServer;
    // }

    //--- Speed controllers
    delete m_pscLeft1;
    delete m_pscLeft2;
    delete m_pscRight1;
    delete m_pscRight2;
    delete m_pscClaw;
    delete m_pscMiniDeploy;
    delete m_pscShoulder;

    //--- Sensors
    delete m_pGyro;
    delete m_pdiLineLeft;
    delete m_pdiLineCenter;
    delete m_pdiLineRight;

    //--- Timers
    delete m_pTimerDrive;
}

//==========================================================================
Robot980 *Robot980::GetInstance()
{
    if (!g_pInstance)
    {
        g_pInstance = new Robot980();
    }
    return g_pInstance;
}

//==========================================================================
//==========================================================================
int Robot980::GetAutonMode()
{
    //--- Get the analog value corresponsind to the autonomous mode to choose
    AnalogModule *pAM = AnalogModule::GetInstance(SLOT_AUTO_MODE);
    int i = pAM->GetValue(CHAN_AUTO_MODE);      // returns 10-bit number

    if (i > 900)
        return 6;
    if (i > 700)
        return 5;
    if (i > 500)
        return 4;
    if (i > 300)
        return 3;
    if (i > 100)
        return 2;

    return 1;
}

//==========================================================================
void Robot980::SetBrakes(bool brakeOnStop)
{
    // kNeutralMode_Jumper, kNeutralMode_Brake, kNeutralMode_Coast
    CANJaguar::NeutralMode mode = brakeOnStop
        ? CANJaguar::kNeutralMode_Brake : CANJaguar::kNeutralMode_Coast;

    m_pscLeft1->ConfigNeutralMode(mode);
    m_pscRight1->ConfigNeutralMode(mode);
    m_pscRight2->ConfigNeutralMode(mode);
    m_pscLeft2->ConfigNeutralMode(mode);
}

//==========================================================================
void Robot980::Drive(float left, float right)
{
    //--- Reset the Timer Drive
    m_pTimerDrive->Reset();

    //--- Set the speed of the left motors
    m_pscLeft1->Set(utils::limit(left));
    m_pscLeft2->Set(utils::limit(left));

    //--- Set the speed of the right motors
    m_pscRight1->Set(utils::limit(-right));
    m_pscRight2->Set(utils::limit(-right));
}

//==========================================================================
void Robot980::setArmSpeed(float speed)
{
    AnalogModule *pAM = AnalogModule::GetInstance(SLOT_ARM_POTENTIOMETER);
    int i = pAM->GetValue(CHAN_ARM_POTENTIOMETER);

    if(speed < 0.0)
    {
        //going up
	if( i > 830 )
	    m_pscShoulder->Set(0.0);
	else
	    m_pscShoulder->Set(utils::limit(speed));
    }
    else
    {
        //going down
	if( i < 440 )
	    m_pscShoulder->Set(0.0);
	else
	    m_pscShoulder->Set(utils::limit(speed));
    }
}

//==========================================================================
char Robot980::GetLineTracker(bool invert /* = false */)
{
    int leftValue   = m_pdiLineLeft->Get()   ? 1 : 0;
    int centerValue = m_pdiLineCenter->Get() ? 1 : 0;
    int rightValue  = m_pdiLineRight->Get()  ? 1 : 0;
    if(invert)
    	return leftValue + centerValue * 2 + rightValue * 4;
    else
    	return leftValue * 4 + centerValue * 2 + rightValue;
}

//==========================================================================
void Robot980::PrintState(void)
{
    utils::message("l/r encoder value: %f %f\n",
        m_pscLeft1->GetPosition(),
        m_pscRight1->GetPosition());
}

//==========================================================================
void Robot980::OpenClaw()
{
    m_pscClaw->Set(0.5);
}

//==========================================================================
void Robot980::CloseClaw()
{
    m_pscClaw->Set(-0.5);
}

//==========================================================================
//float Robot980::GetAngle(void)
//{
//   return 1.0;
//}
