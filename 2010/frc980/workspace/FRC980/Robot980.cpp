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

    // Timer used for debugging
    , m_pTimer(new Timer)

    , m_pSrvPan(new Servo(DSC_SLOT, CAMERA_CHAN_PAN))
    , m_pSrvTilt(new Servo(DSC_SLOT, CAMERA_CHAN_TILT))
    , m_pVideoServer(NULL)
{
    // pi * diameter * gear ratio / encoder ticks / in/ft
    m_pEncDrvLeft->SetDistancePerPulse(M_PI * 6 * GEAR_RATIO / 250 / 12);
    m_pEncDrvRight->SetDistancePerPulse(M_PI * 6 * GEAR_RATIO / 250 / 12);

    m_pEncDrvLeft->Start();
    m_pEncDrvRight->Start();

    m_pTimer->Reset();
    m_pTimer->Start();

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

    // camera system
    delete m_pSrvPan;
    delete m_pSrvTilt;

    delete m_pVideoServer;
}

int Robot980::GetAutonMode()
{
    AnalogModule *pAM = AnalogModule::GetInstance(SLOT_AUTO_MODE);
    int i = pAM->GetValue(CHAN_AUTO_MODE);

    return i;
}

// 1 = forward, -1 = backwards
void Robot980::Drive(float left, float right)
{
    m_pTimer->Reset();

    m_pscLeft->Set(left);
    m_pscRight->Set(right);
}
