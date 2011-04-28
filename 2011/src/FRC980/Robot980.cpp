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

    , m_pscMiniAlign(new CANJaguar(CAN_MINIALIGN))
    , m_pscClaw(new CANJaguar(CAN_ARM_CLAW))

      //--- Victors
    , m_pscShoulder(new Victor(DSC_SLOT, CHAN_PWM_SHOULDER))
    , m_pscMiniDeploy(new Victor(DSC_SLOT, CHAN_MINIDEPLOY))

      //--- Lights
    , m_pdoLightTriangle(new DigitalOutput(DSC_SLOT, CHAN_LIGHT_TRIANGLE))
    , m_pdoLightCircle(new DigitalOutput(DSC_SLOT, CHAN_LIGHT_CIRCLE))
    , m_pdoLightSquare(new DigitalOutput(DSC_SLOT, CHAN_LIGHT_SQUARE))

      //--- Sensors
#ifdef USE_GYRO
    , m_pGyro(new Gyro(SLOT_GYRO, CHAN_GYRO))
#endif
    , m_pdiLineLeft(new DigitalInput(DSC_SLOT, CHAN_LINE_LEFT))
    , m_pdiLineCenter(new DigitalInput(DSC_SLOT, CHAN_LINE_CENTER))
    , m_pdiLineRight(new DigitalInput(DSC_SLOT, CHAN_LINE_RIGHT))
    , m_pacAutonSwitch(new AnalogChannel(ANALOG_SLOT, CHAN_AUTO_MODE))
    , m_pacArmPosition(new AnalogChannel(ANALOG_SLOT, CHAN_ARM_POTENTIOMETER))

      //--- PIDs
    , m_pidArm(new PIDController(POT_PID_P,POT_PID_I,POT_PID_D,m_pacArmPosition, m_pscShoulder))

      //--- Timers
    , m_pTimerDrive(new Timer)
    , m_pTimerClaw(new Timer)

      //--- Notifiers
#ifdef USE_NOTIFIER
    , m_pNotifierClaw(new Notifier(Robot980::CheckClaw, this))
#endif

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
    
    //--- Set up PID loops
    m_pidArm->SetInputRange(POT_LOWER_LIMIT,POT_UPPER_LIMIT);
    m_pidArm->SetTolerance(POT_TOLERANCE /*%*/);
    m_pidArm->SetOutputRange(-0.1,1.0);

    //--- Define Drive Timer
    m_pTimerDrive->Reset();
    m_pTimerDrive->Start();

    //--- Start Claw Timer
    m_pTimerClaw->Reset();
    m_pTimerClaw->Start();

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
    delete m_pscMiniAlign;
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
void Robot980::SetPosition(int target) {
    if(!m_pidArm->IsEnabled())
    {
        m_pidArm->Enable();
        utils::message("enabling PID");
    }

    m_pidArm->SetSetpoint(target);
}

//==========================================================================

void Robot980::SetArmSpeed(float speed) {
	utils::message("setting arm speed to %f", speed);
    if(m_pidArm->IsEnabled())
    {
        m_pidArm->Disable();
        utils::message("disabling PID");
        return;
    }

    if ((m_pacArmPosition->GetValue() > POT_VERTICAL) && (speed > 0))
        m_pscShoulder->Set(0.0);
    else
        m_pscShoulder->Set(speed);
}

//==========================================================================

int Robot980::GetPosition() {
    return m_pidArm->Get();

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
void Robot980::LightLED(LED_t led)
{
    switch(led)
    {
    case LED_OFF:
        m_pdoLightTriangle->Set(0);
        m_pdoLightCircle->Set(0);
        m_pdoLightSquare->Set(0);
        break;
    case LED_TRIANGLE:
        m_pdoLightTriangle->Set(1);
        break;
    case LED_CIRCLE:
        m_pdoLightCircle->Set(1);
        break;
    case LED_SQUARE:
        m_pdoLightSquare->Set(1);
        break;
    }
}

//==========================================================================
float Robot980::GetRightEncoder()
{
    return m_pscRight1->GetPosition() * 21.8;//3.14159 * WHEEL_DIAMETER;
}

//==========================================================================
float Robot980::GetLeftEncoder()
{
    return m_pscLeft1->GetPosition() * 21.8;//3.14159 * WHEEL_DIAMETER;
}

//==========================================================================
void Robot980::PrintState(void)
{
    int i = m_pacArmPosition->GetValue();

    utils::message("potentiometer: %d\n", i);
}

//==========================================================================
void Robot980::OpenClaw(float speed)
{    
    m_pscClaw->Set(speed);
    m_pTimerClaw->Reset();
#ifdef USE_NOTIFIER
    m_pNotifierClaw->Stop(); 
    m_pNotifierClaw->StartPeriodic(0.1);
#endif
}

void Robot980::CloseClaw(float speed)
{
    m_pscClaw->Set(-speed);
    m_pTimerClaw->Reset();
#ifdef USE_NOTIFIER
    m_pNotifierClaw->Stop();
    m_pNotifierClaw->StartPeriodic(0.1);
#endif
}

void Robot980::RunClaw(float speed)
{    
    m_pscClaw->Set(speed);
}

float Robot980::GetClawTimer()
{
    return m_pTimerClaw->Get();
}

float Robot980::GetClawCurrent()
{
    return m_pscClaw->GetOutputCurrent();
}

void Robot980::CheckStopClaw()
{
    if (    GetClawTimer() > 0.3
         && GetClawCurrent() > 35.0
         && m_pscClaw->GetOutputVoltage() < 0.0 //closing claw
        )
    {
        utils::message("Claw disabled at current=%f", GetClawCurrent());
        RunClaw(0.0);
    }

    if (GetClawTimer() > 1.5 )
    {
        RunClaw(0.0);
    }
}

/*static*/
void Robot980::CheckClaw(void* pvRobot)
{
    utils::message("CheckClaw");
    Robot980* pRobot=static_cast<Robot980*>(pvRobot);

    float t = pRobot->m_pTimerClaw->Get();

    if (t < 0.25)
    {
        utils::message("Do nothing");
        // do nothing: Let claw run for 0.25 seconds
    }
    else if (t < 1.0)
    {
        utils::message("Less than one second");
        // From 0.25 seconds to 1 second,
        //     Stop the claw if current exceeds the limit
        if (pRobot->m_pscClaw->GetOutputCurrent() > 15.0)
        {
            utils::message("Stopped");
            pRobot->m_pscClaw->Set(0.0);
//            pRobot->m_pNotifierClaw->Stop();
        }
    }
    else
    {
        utils::message("More than one second");
        // After 1 second, stop the claw
        pRobot->m_pscClaw->Set(0.0);
//        pRobot->m_pNotifierClaw->Stop();
    }
}

//==========================================================================

void Robot980::Deploy(float speed)
{
    m_pscMiniDeploy->Set(speed);
}

void Robot980::Align(float speed)
{
    m_pscMiniAlign->Set(speed);
}

//==========================================================================
//float Robot980::GetAngle(void)
//{
//   return 1.0;
//}
