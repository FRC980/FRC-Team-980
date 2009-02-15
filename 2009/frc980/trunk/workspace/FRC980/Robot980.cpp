#include <WPILib.h>
#include <stdbool.h>
#include <AxisCamera.h>
#include <PCVideoServer.h>

#include "Robot980.h"

static Robot980* g_pInstance = NULL;

Robot980* Robot980::GetInstance()
{
    if (!g_pInstance)
    {
        g_pInstance = new Robot980();
    }
    return g_pInstance;
}

Robot980::Robot980()
{
    // left and right drive motors
    m_pscLeft  = new Jaguar(SLOT_PWM_LEFT, CHAN_PWM_LEFT);
    m_pscRight = new Jaguar(SLOT_PWM_RIGHT,CHAN_PWM_RIGHT);
    // lower and upper drive belts
    m_pscLowerBelt = new Jaguar(SLOT_PWM_LOWER, CHAN_PWM_LOWER);
    m_pscUpperBelt = new Jaguar(SLOT_PWM_UPPER, CHAN_PWM_UPPER);
    // ball release
    m_pscFlap = new Jaguar(SLOT_PWM_FLAP, CHAN_PWM_FLAP);


    // sensors
    m_pGyro = new Gyro(SLOT_GYRO, CHAN_GYRO);

    // encoders on the drive wheels
    m_pEncDrvLeft  = new Encoder(SLOT_ENC_DRV_LEFT,CHAN_ENC_DRV_LEFT_A,
                                 SLOT_ENC_DRV_LEFT,CHAN_ENC_DRV_LEFT_B,
                                 false);
    m_pEncDrvRight = new Encoder(SLOT_ENC_DRV_RIGHT,CHAN_ENC_DRV_RIGHT_A,
                                 SLOT_ENC_DRV_RIGHT,CHAN_ENC_DRV_RIGHT_B,
                                 false);

    // encoders on the follow wheels
    m_pEncFollowLeft  = new Encoder(SLOT_ENC_FOLLOW_LEFT,
                                    CHAN_ENC_FOLLOW_LEFT_A,
                                    SLOT_ENC_FOLLOW_LEFT,
                                    CHAN_ENC_FOLLOW_LEFT_B,
                                    false);
    m_pEncFollowRight = new Encoder(SLOT_ENC_FOLLOW_RIGHT,
                                    CHAN_ENC_FOLLOW_RIGHT_A,
                                    SLOT_ENC_FOLLOW_RIGHT,
                                    CHAN_ENC_FOLLOW_RIGHT_B,
                                    false);

    // "Smart Drive" handles PID, slipping, etc
    m_psdLeft  = new SmartDrive(m_pscLeft, m_pEncDrvLeft, m_pEncFollowLeft);
    m_psdRight = new SmartDrive(m_pscRight, m_pEncDrvRight, m_pEncFollowRight);



    ParticleAnalysisReport par1, par2; // particle analysis reports
    memset(&par1, 0, sizeof(ParticleAnalysisReport));
    memset(&par2, 0, sizeof(ParticleAnalysisReport));

    /* image data for tracking - override default parameters if needed */
    /* recommend making PINK the first color because GREEN is more
     * susceptible to hue variations due to lighting type so may
     * result in false positives */

    // PINK
    sprintf(m_tdPink.name, "PINK");
    m_tdPink.hue.minValue = 220;
    m_tdPink.hue.maxValue = 255;
    m_tdPink.saturation.minValue = 75;
    m_tdPink.saturation.maxValue = 255;
    m_tdPink.luminance.minValue = 85;
    m_tdPink.luminance.maxValue = 255;
    // GREEN
    sprintf(m_tdGreen.name, "GREEN");
    m_tdGreen.hue.minValue = 55;
    m_tdGreen.hue.maxValue = 125;
    m_tdGreen.saturation.minValue = 58;
    m_tdGreen.saturation.maxValue = 255;
    m_tdGreen.luminance.minValue = 92;
    m_tdGreen.luminance.maxValue = 255;



    /* start the CameraTask  */
    StartCameraTask(20/*FPS*/, 0, k320x240, ROT_180);
    m_pVideoServer = new PCVideoServer;

}

Robot980::~Robot980()
{
    delete m_psdLeft;
    delete m_psdRight;
    delete m_pEncDrvLeft;
    delete m_pEncDrvRight;
    delete m_pEncFollowLeft;
    delete m_pEncFollowRight;
    delete m_pscLeft;
    delete m_pscRight;
    delete m_pscLowerBelt;
    delete m_pscUpperBelt;
    delete m_pscFlap;
    delete m_pGyro;
    delete m_pVideoServer;
}

// 1 = forward, -1 = backwards
void Robot980::Drive(float left, float right)
{
    m_psdLeft->Set(left);
    m_psdRight->Set(right);
}

// 1 = in @ base, up and out
void Robot980::RunBelts(float lower, float upper)
{
    m_pscLowerBelt->Set(lower);
    m_pscUpperBelt->Set(upper);
}

void Robot980::Flap(bool open)
{
    m_pscFlap->Set(open ? 0.5 : -0.5);
}

bool FindTrailer(DriverStation::Alliance alliance)
{
    return false;
}
