#include <WPILib.h>
#include <PCVideoServer.h>
#include <Timer.h>
#include <math.h>
#include <stdbool.h>

#include "ReversableJaguar.h"
#include "Robot980.h"
#include "numbers.h"
#include "utils.h"

static Robot980* g_pInstance = NULL;

Robot980* Robot980::GetInstance()
{
    if (!g_pInstance)
    {
        g_pInstance = new Robot980();
    }
    return g_pInstance;
}

#define ENC_SCALE   CounterBase::k1X

Robot980::Robot980()
    // left and right drive motors
    : m_pscLeft(new ReversableJaguar(DSC_SLOT, CHAN_PWM_LEFT, true))
    , m_pscRight(new Jaguar(DSC_SLOT, CHAN_PWM_RIGHT))

    , m_pscRoller(new Jaguar(DSC_SLOT, CHAN_PWM_ROLLER))
    , m_pscArmer(new Jaguar(DSC_SLOT, CHAN_PWM_ARMER)) // may be victor?
    , m_pscFire(new Jaguar(DSC_SLOT, CHAN_PWM_FIRE))   // may be victor?

    , m_pscLift(new Jaguar(DSC_SLOT, CHAN_PWM_LIFT)) // may be victor?

    // sensors
    , m_pGyro(new Gyro(SLOT_GYRO, CHAN_GYRO))

    // encoders on the drive wheels
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

    , m_pTimerDrive(new Timer)  // Timer used for debugging
    , m_pTimerFire(new Timer)   // can only fire once every 2 seconds

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

    /* start the CameraTask  */
    m_pVideoServer = new PCVideoServer;

    // tell SensorBase about us
    AddToSingletonList();
}

Robot980::~Robot980()
{
    // encoders
    delete m_pEncDrvLeft;
    delete m_pEncDrvRight;

    // speed controllers
    delete m_pscLeft;
    delete m_pscRight;
    delete m_pscRoller;
    delete m_pscArmer;
    delete m_pscFire;
    delete m_pscLift;

    // sensors
    delete m_pGyro;

    delete m_pTimerDrive;
    delete m_pTimerFire;

    // camera system
    delete m_pSrvPan;
    delete m_pSrvTilt;

    delete m_pVideoServer;
}

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

// 1 = forward, -1 = backwards
void Robot980::Drive(float left, float right)
{
    m_pTimerDrive->Reset();

    m_pscLeft->Set(left);
    m_pscRight->Set(right);

    // Roller
    if (left + right > 0) // going forward
        m_pscRoller->Set(0);
    else
    {
        // Roller runs from a CIM through an 11:3 gearbox.  The Roller is
        // a 3" dia; the ball is 9" dia, and the wheels are 6" dia.  This
        // means RPM of ball is 2/3 RPM of wheels, and roller is 3x RPM of
        // ball, making roller 2x RPM of wheels.  We then add an extra
        // 10%.
        double speed = (left + right) / 2;
        speed *= 2 * 1.1 * (ROLLER_GEARBOX) / (GEARBOX_RATIO);
        m_pscRoller->Set(limit(speed));
    }
}

bool Robot980::Kick()
{
    if (! CanKick())
        return false;

    return false;
}

bool Robot980::CanKick()
{

    return false;
}
