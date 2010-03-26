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

static const char* szArmingArr[] = {
    "UNKNOWN",                /* 0 */
    "READY_TO_FIRE",          /* 1 */
    "FIRED",                  /* 2 */
    "WINDING",                /* 3 */
    "WOUND",                  /* 4 */
    "UNWINDING",              /* 5 */
};

//==========================================================================
//==========================================================================
Robot980::Robot980()
    //--- Jaguars (Encoders attached directly to Jaguars)
    //    NOTE: CANNOT RUN IN kSpeed MODE UNLESS ENCODERS ATTACHED
    // left and right drive motors
    : m_pscLeft_cim1(new CANJaguar(CAN_LEFT_CIM1))   //, CANJaguar::kSpeed))
    , m_pscLeft_cim2(new CANJaguar(CAN_LEFT_CIM2))   //, CANJaguar::kSpeed))
    , m_pscRight_cim1(new CANJaguar(CAN_RIGHT_CIM1)) //, CANJaguar::kSpeed))
    , m_pscRight_cim2(new CANJaguar(CAN_RIGHT_CIM2)) //, CANJaguar::kSpeed))
    // roller and lift motors
    , m_pscRoller_fp(new CANJaguar(CAN_ROLLER_FP))   //, CANJaguar::kSpeed))
    //, m_pscLift(new CANJaguar(CAN_LIFT))             //, CANJaguar::kSpeed))
    
    //--- Victors
    , m_pscArm_win(new Victor(DSC_SLOT, CHAN_PWM_ARM))
    , m_pscFire_win(new Victor(DSC_SLOT, CHAN_PWM_FIRE))
    
    //--- Sensors
    //, m_pGyro(new Gyro(SLOT_GYRO, CHAN_GYRO))
    , m_pdiArmed_switch(new DigitalInput(DSC_SLOT, CHAN_LIMIT_ARMED))
    , m_pdiFireCam_switch(new DigitalInput(DSC_SLOT, CHAN_LIMIT_FIRE_READY))
    , m_pdiWinch_switch(new DigitalInput(DSC_SLOT, CHAN_LIMIT_WINCH_COUNTER))
    
    //--- Timers
    , m_pTimerDrive(new Timer)
    , m_pTimerFire(new Timer)
    , m_pTimerWinch(new Timer)
    , m_iUnwindCount(0)

    , m_pNoteWinch(NULL)

    //--- State variables
    , m_armingState(UNKNOWN)
    , m_bArmingEnable(true)

    //, m_pVideoServer(NULL)
//    , m_pVideoServer(new PCVideoServer)
{
    //--- Set the PID Values for each Jag in Speed or Current Mode
    //double kp = 0.35;
    //double ki = 0.003;
    //double kd = 0.001;
    //m_pscLeft_cim1->SetPID( kp,  ki,  kd);
    //m_pscLeft_cim2->SetPID( kp,  ki,  kd);
    //m_pscRight_cim1->SetPID( kp,  ki,  kd);
    //m_pscRight_cim2->SetPID( kp,  ki,  kd);
    //m_pscRoller_cim->SetPID( kp,  ki,  kd);

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
    if (m_pTimerFire)
    {
        m_pTimerFire->Reset();
        m_pTimerFire->Start();
    }

    m_pTimerWinch->Reset();
    m_pTimerWinch->Start();

    // setup interrupts
    if (1)
    {
        m_pdiFireCam_switch->RequestInterrupts(Robot980::CallStopCam, this);
        m_pdiFireCam_switch->SetUpSourceEdge(false, true);
        m_pdiFireCam_switch->EnableInterrupts();

        m_pdiArmed_switch->RequestInterrupts(Robot980::ArmedIntHandler, this);
        m_pdiArmed_switch->SetUpSourceEdge(true, true);
        m_pdiArmed_switch->EnableInterrupts();

        m_pdiWinch_switch->RequestInterrupts(Robot980::WinchIntHandler, this);
        m_pdiWinch_switch->SetUpSourceEdge(true, true);
        m_pdiWinch_switch->EnableInterrupts();
    }

    // figure out what our UNKNOWN state should be
    DoWinchStateMachineTransition();

    m_pNoteWinch = new Notifier(Robot980::CallWinchStateMachineTimer, this);
    m_pNoteWinch->StartPeriodic(0.010);

    //--- Tell SensorBase about us
    AddToSingletonList();

/*
    if (m_pVideoServer)
    {
        AxisCamera::GetInstance().WriteRotation(AxisCamera::kRotation_180);
        AxisCamera::GetInstance().WriteResolution(AxisCamera::kResolution_320x240);
        AxisCamera::GetInstance().WriteMaxFPS(15);
        m_pVideoServer->Start();
    }
*/
}

//==========================================================================
Robot980::~Robot980()
{
/*
    if (m_pVideoServer)
    {
        m_pVideoServer->Stop();
        delete m_pVideoServer;
    }
*/

    //--- Speed controllers
    delete m_pscLeft_cim1;
    delete m_pscLeft_cim2;
    delete m_pscRight_cim1;
    delete m_pscRight_cim2;

    delete m_pscRoller_fp;
    //delete m_pscLift;

    delete m_pscArm_win;
    delete m_pscFire_win;

    //--- Sensors
    //delete m_pGyro;
    delete m_pdiArmed_switch;
    delete m_pdiFireCam_switch;
    delete m_pdiWinch_switch;

    //--- Timers
    delete m_pTimerDrive;
    delete m_pTimerFire;
    delete m_pTimerWinch;
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
        return 5;
    if (i > 700)
        return 4;
    if (i > 500)
        return 3;
    if (i > 300)
        return 2;
    if (i > 100)
        return 1;

    return 0;
}

//==========================================================================
void Robot980::SetBrakes(bool brakeOnStop)
{
    // kNeutralMode_Jumper, kNeutralMode_Brake, kNeutralMode_Coast
    CANJaguar::NeutralMode mode = brakeOnStop
        ? CANJaguar::kNeutralMode_Brake
        : CANJaguar::kNeutralMode_Coast;

    m_pscLeft_cim1->ConfigNeutralMode(mode);
    m_pscRight_cim1->ConfigNeutralMode(mode);
    m_pscRight_cim2->ConfigNeutralMode(mode);
    m_pscLeft_cim2->ConfigNeutralMode(mode);
}

//==========================================================================
void Robot980::Drive(float left, float right, float roller)
{
    //--- Reset the Timer Drive
    m_pTimerDrive->Reset();

    //--- Set the speed of the left motors
    m_pscLeft_cim1->Set(utils::limit(left));
    m_pscLeft_cim2->Set(utils::limit(left));

    //--- Set the speed of the right motors
    m_pscRight_cim1->Set(utils::limit(- right));
    m_pscRight_cim2->Set(utils::limit(- right));

    //--- Set the speed of the roller motor
    m_pscRoller_fp->Set(utils::limit(roller));
}

//==========================================================================
void Robot980::Drive(float left, float right)
{
    //--- Set the speed of the roller motor based upon the forward/back speed
    //    The forward speed here represents a direct relation to the y-axis
    //    input of the command joystick.
    //
    //    Ex: fForwardSpeed = utils::limit(((y-x) + (y+x))/2)
    //                      = utils::limit((y-x+y+x)/2)
    //                      = utils::limit((2*y)/2)
    //                      = utils::limit(y)
    //
    float fForwardSpeed = utils::limit((left + right) / 2);

    /* IMPORTANT!
     *
     * IF THE ROLLER IS GOING OPPOSITE THAN DESIRED THE
     * fForwardSpeed VARIABLE NEEDS TO BE REVERSED, BUT
     * NOT THE ROLLER DRIVE SPEED.
     *
     */

    //--- Set when going forward
    if (fForwardSpeed > 0)
    {
        this->Drive(left, right, 0.0);
    }
    //--- Set a constant speed if not moving forward
    else if (fForwardSpeed == 0)
    {
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

//==========================================================================
void Robot980::RunWinch(float speed) {
    m_pscArm_win->Set(utils::limit(speed));
}

//==========================================================================
bool Robot980::KickerReady(void)
{
    return (m_pdiArmed_switch->Get() == SW_CLOSED);

    return ((READY_TO_FIRE == m_armingState) &&
            (m_pdiArmed_switch->Get() == SW_CLOSED));
}

//==========================================================================
void Robot980::ArmKicker(void)
{
    if ((FIRED == m_armingState) &&
        (m_pdiArmed_switch->Get() == SW_OPEN))
    {
        m_armingState = WINDING;
        this->RunWinchState();
    }
}

void Robot980::Unwind(void)
{
    m_armingState = WOUND;
    RunWinchState();
    m_armingState = UNWINDING;
    RunWinchState();
}

void Robot980::PrintState(void)
{
    utils::message("armed: %d  cam: %d  winch: %d  state: %d / %s\n",
                   m_pdiArmed_switch->Get(),
                   m_pdiFireCam_switch->Get(),
                   m_pdiWinch_switch->Get(),
                   (int)m_armingState,
                   szArmingArr[m_armingState]);
}

//==========================================================================
bool Robot980::FireKicker(void)
{
    if (this->KickerReady())
    {
        //--- Reset the firing timer
        if (m_pTimerFire)
        {
            m_pTimerFire->Reset();
            m_pTimerFire->Start();
        }

        //--- Run the Kick motor to release the kicker
        m_pscFire_win->Set(1.0);
        m_armingState = FIRED;
        this->RunWinchState();

        return true;
    }

    return false;
}

//==========================================================================
void Robot980::ArmingEnable()
{
    m_bArmingEnable = true;
    RunWinchState();
}

//==========================================================================
void Robot980::ArmingDisable()
{
    m_bArmingEnable = false;
    m_pscArm_win->Set(0);
}

//==========================================================================
void Robot980::HandleFiring(void)
{
    static bool bOldCamState = SW_OPEN;

    if (m_pTimerFire->Get() > 0.100)
    {
        if (m_pdiFireCam_switch->Get() == SW_CLOSED)
        {
            m_pscFire_win->Set(0);

            ArmKicker();
        }
    }

    bOldCamState = m_pdiFireCam_switch->Get();
}

//==========================================================================
#define WIND_SPEED      1.0
#define UNWIND_SPEED    1.0
#define UNWIND_TIME     2.0     /* in seconds */
#define UNWIND_COUNT    10      /* both edges, 2 sensors per rotation */

void Robot980::DoWinchStateMachineTransition()
{
    // Some event has occurred -- switch triggered, timeout, startup --
    // and we must now figure out what stateto transition into
    switch (m_armingState)
    {
    case UNKNOWN:
    {
        // At power-up, figure out the initial state of the kicker.
        //
        // REVIEW: Assume that it is NOT in a "bad" state, such as only
        // partially unwound.
        //
        // likely states are:
        // - READY_TO_FIRE
        // - WOUND
        // - FIRED

        if (m_pdiArmed_switch->Get() == SW_OPEN)
            m_armingState = FIRED;
        else
            m_armingState = READY_TO_FIRE;

        // REVIEW: There is no way to tell the difference between
        // READY_TO_FIRE vs. WOUND.
    }
    break;

    case READY_TO_FIRE:
    {
        if (m_pdiArmed_switch->Get() == SW_OPEN)
            m_armingState = FIRED;
    }
    break;

    case FIRED:
    {
        // nothing to do here
    }
    break;

    case WINDING:
    {
        // if we've armed, enter the wound state
        if ((SW_CLOSED == m_pdiArmed_switch->Get()))
        {
            m_armingState = WOUND;
        }
    }
    break;
    
    case WOUND:
    {
    }
    break;

    case UNWINDING:
    {
        m_iUnwindCount++;

#ifdef UNWIND_COUNT
        if (m_iUnwindCount >= UNWIND_COUNT)
        {
            m_armingState = READY_TO_FIRE;
        }
#endif
        utils::message("unwind count: %d  time: %.4f\n",
                       m_iUnwindCount, m_pTimerWinch->Get());
    }
    break;
    }

    static arming_t lastState = UNKNOWN;
    if (lastState != m_armingState)
    {
        PrintState();
        lastState = m_armingState;
    }
}

//==========================================================================
void Robot980::RunWinchState()
{
    if (!m_bArmingEnable)
    {
        utils::message("winch disabled\n");
        return;
    }

    // second, do whatever we need to do in our new state
    switch (m_armingState)
    {
    case UNKNOWN:
    {
        // We should never be "running" an the UNKNOWN state.  If we are,
        // we need to invoke a transition to figure out what state we
        // should be in.
        DoWinchStateMachineTransition();
    }
    break;

    case READY_TO_FIRE:
    {
        // winch is unwound, and kicker is armed
        m_pscArm_win->Set(0);
    }
    break;

    case FIRED:
    {
        // nothing to do here
    }
    break;

    case WINDING:
    {
        m_pscArm_win->Set(WIND_SPEED);
    }
    break;
    
    case WOUND:
    {
        m_pscArm_win->Set(0);
        m_pTimerWinch->Start();
        m_pTimerWinch->Reset();
        m_iUnwindCount = 0;
    }
    break;

    case UNWINDING:
    {
        m_pscArm_win->Set(- UNWIND_SPEED);
        if (m_pTimerWinch->Get() >= UNWIND_TIME)
        {
            m_pscArm_win->Set(0);
            m_armingState = READY_TO_FIRE;
        }
    }
    break;
    }
}

//==========================================================================
void Robot980::CallStopCam(tNIRIO_u32 mask, void*) // static
{
    Robot980::GetInstance()->HandleFiring();
}

//==========================================================================
void Robot980::ArmedIntHandler(tNIRIO_u32 mask, void*) // static
{
    Robot980::GetInstance()->DoWinchStateMachineTransition();
    Robot980::GetInstance()->RunWinchState();
}

void Robot980::WinchIntHandler(tNIRIO_u32 mask, void*) // static
{
    // Attempt to debounce input -- if we see multiple changes within 1
    // millisecond, only count the first one.
    
    static Timer* m_pTimerDebounce = NULL;
    if (!m_pTimerDebounce)
    {
        m_pTimerDebounce = new Timer;
        m_pTimerDebounce->Start();
        m_pTimerDebounce->Reset();
        Wait(0.005);
    }

    if (m_pTimerDebounce->Get() > 0.001)
    {
        Robot980::GetInstance()->DoWinchStateMachineTransition();
        Robot980::GetInstance()->RunWinchState();
    }
    m_pTimerDebounce->Start();
    m_pTimerDebounce->Reset();
}

//==========================================================================
void Robot980::CallWinchStateMachineTimer(void*) // static
{
    Robot980::GetInstance()->RunWinchState();
}

//==========================================================================
//void Robot980::Lift(void)
//{
//
//}

//==========================================================================
//==========================================================================
//float Robot980::GetAngle(void)
//{
//   return 1.0;
//}
