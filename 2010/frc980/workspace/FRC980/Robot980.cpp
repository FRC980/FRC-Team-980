#include <WPILib.h>
#include <NetworkCommunication/FRCComm.h>
#include <Timer.h>
#include <Vision/PCVideoServer.h>
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
    "WINDING",                /* 2 */
    "START_UNWINDING",        /* 3 */
    "UNWINDING",              /* 4 */
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
    , m_pTimerFire(NULL)
//    , m_pTimerFire(new Timer)

    , m_pTimerUnwind(new Timer)
    , m_pnWinchPolling(new Notifier(Robot980::CallHandleAutomatic, this))
    
    //--- State variables
    , m_armingState(UNKNOWN)
    , m_bArmingEnable(true)

    , m_pVideoServer(NULL)
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

    m_pTimerUnwind->Reset();
    m_pTimerUnwind->Start();
    
    //--- Tell SensorBase about us
    AddToSingletonList();

    m_pnWinchPolling->StartPeriodic(0.010);

    if (m_pVideoServer)
    {
        AxisCamera::GetInstance().WriteRotation(AxisCamera::kRotation_180);
        AxisCamera::GetInstance().WriteResolution(AxisCamera::kResolution_320x240);
        AxisCamera::GetInstance().WriteMaxFPS(15);
        m_pVideoServer->Start();
    }
}

//==========================================================================
Robot980::~Robot980()
{
    if (m_pVideoServer)
    {
        m_pVideoServer->Stop();
        delete m_pVideoServer;
    }

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
    delete m_pTimerUnwind;

    delete m_pnWinchPolling;
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
    m_pscLeft_cim2->ConfigNeutralMode(mode);
    m_pscRight_cim1->ConfigNeutralMode(mode);
    m_pscRight_cim2->ConfigNeutralMode(mode);
}

//==========================================================================
void Robot980::Drive(float left, float right, float roller)
{
    //--- Reset the Timer Drive
    m_pTimerDrive->Reset();

    //--- Set up utilities to use
    utils u;

    //--- Set the speed of the left motors
    m_pscLeft_cim1->Set(u.limit(left * REVERSE_DRIVE));
    m_pscLeft_cim2->Set(u.limit(left * REVERSE_DRIVE));

    //--- Set the speed of the right motors
    m_pscRight_cim1->Set(u.limit(right));
    m_pscRight_cim2->Set(u.limit(right));

    //--- Set the speed of the roller motor
    m_pscRoller_fp->Set(u.limit(roller));
}

//==========================================================================
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
    if (fForwardSpeed > 0)
    {
        this->Drive(left, right, 0.0);
    }
    //--- Set a constant speed if not moving forward
    else if (fForwardSpeed == 0)
    {
        this->Drive(left, right, 0.5 * REVERSE_DRIVE);
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
bool Robot980::KickerArmed(void)
{
    return (m_pdiArmed_switch->Get() == SW_CLOSED);
}

//==========================================================================
void Robot980::ArmKicker(void)
{
    if ((READY_TO_FIRE == m_armingState) &&
        (m_pdiArmed_switch->Get() == SW_OPEN))
    {
        m_armingState = WINDING;
        this->HandleArming();
    }
}

//==========================================================================
void Robot980::FireKicker(void)
{
    if (this->KickerArmed())
    {
        //--- Reset the firing timer
        if (m_pTimerFire)
        {
            m_pTimerFire->Reset();
            m_pTimerFire->Start();
        }

        //--- Run the Kick motor to release the kicker
        m_pscFire_win->Set(0.35);
    }
}

//==========================================================================
void Robot980::ArmingEnable()
{
    m_bArmingEnable = true;
}

//==========================================================================
void Robot980::ArmingDisable()
{
    m_bArmingEnable = false;
}

//==========================================================================
void Robot980::SetWinch(float speed)
{
    m_pscArm_win->Set(speed);
}

//==========================================================================
void Robot980::HandleFiring(void)
{
    static bool bOldCamState = SW_OPEN;

    if ((bOldCamState == SW_OPEN) &&
        (m_pdiFireCam_switch->Get() == SW_CLOSED))
    {
        m_pscFire_win->Set(0);
    }

    bOldCamState = m_pdiFireCam_switch->Get();
}

//==========================================================================
void Robot980::HandleArming()
{
#define WIND_SPEED      0.4
#define UNWIND_SPEED    0.2
#define UNWIND_TIME     2.5     /* in seconds */

    if (!m_bArmingEnable)
    {
        return;
    }

    static arming_t lastState = UNKNOWN;

    if (lastState != m_armingState)
    {
        char error[256];
        sprintf(error, "armed: %d  cam: %d  winch: %d  state: %d / %s\n",
                m_pdiArmed_switch->Get(),
                m_pdiFireCam_switch->Get(),
                m_pdiWinch_switch->Get(),
                (int)m_armingState,
                szArmingArr[m_armingState]);
        setErrorData(error, strlen(error), 100);

        lastState = m_armingState;
    }

    switch (m_armingState)
    {
    case UNKNOWN:
    {
        // At power-up, figure out the initial state of the kicker.
        //
        // REVIEW: This needs to be smarter, as there are more possible
        // states.

        // REVIEW: For now, assume that it is ready to fire -- if it
        // isn't, then either auton mode or the operator needs to arm it.
        // Assume that it is NOT in a "bad" state, such as only partially
        // unwound.

        m_armingState = READY_TO_FIRE;
    }
    break;

    case READY_TO_FIRE:
    {
        // winch is unwound, and kicker is armed
        m_pscArm_win->Set(0);
    }
    break;

    // we need to wind the winch in until the armed switch is closed;
    // ideally this should be exactly two loops; we do the first one
    // "fast" and the second one "slow".  After winding in, we need to
    // unwind two loops.  In theory, the winch switch and the armed switch
    // should both trigger at the same time.  In practice, this may not be
    // possible to make happen.  We want to stop winding based only on the
    // armed switch.  We have a "preunwind" state in hopes that we are
    // slightly past the winch trigger when armed, rather than slightly
    // before it.  We "pre unwind" for a partial loop, then unwind "fast"
    // for one loop and finally unwind "slow" for one loop.  Finally, we
    // are ready to fire.
    case WINDING:
    {
        m_pscArm_win->Set(WIND_SPEED);
        // if we've armed, start unwinding
        if ((SW_CLOSED == m_pdiArmed_switch->Get()))
        {
            m_armingState = START_UNWINDING;
        }
    }
    break;

    case START_UNWINDING:
        m_pTimerUnwind->Start();
        m_pTimerUnwind->Reset();
        m_armingState = UNWINDING;
        // fall through

    case UNWINDING:
    {
        m_pscArm_win->Set(UNWIND_SPEED * REVERSE_DRIVE);

        if (m_pTimerUnwind->Get() >= UNWIND_TIME)
        {
            m_pscArm_win->Set(0);
            m_armingState = READY_TO_FIRE;
        }
    }
    break;
    }
}

//==========================================================================
void Robot980::DebugPrinting()
{
    AnalogModule* pAM = AnalogModule::GetInstance(SLOT_AUTO_MODE);

    char string1[256] = {0};
    char string2[256] = {0};

    sprintf(string1, "ANA:");
    for (int chan = 1 ; chan <= 8 ; chan++)
    {
        int i = pAM->GetValue(chan);      // returns 10-bit number
        sprintf(string2, "  %d:%04d", chan, i);
        strncat(string1, string2, 255-strlen(string2));
    }
    sprintf(string2, "\n");
    strncat(string1, string2, 255-strlen(string2));
    setErrorData(string1, strlen(string1), 100);
}

//==========================================================================
void Robot980::HandleAutomatic()
{
    this->HandleFiring();
    this->HandleArming();
    // In the future, we may have additional automatic functionality (eg a
    // compressor)

    static int i = 0;

    if (++i >= 10)
    {
        i = 0;
        DebugPrinting();
    }
}

//==========================================================================
void Robot980::CallHandleAutomatic(void*) // static
{
    Robot980::GetInstance()->HandleAutomatic();
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

