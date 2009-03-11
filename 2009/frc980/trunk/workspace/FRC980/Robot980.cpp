#include <WPILib.h>
#include <AxisCamera.h>
#include <PCVideoServer.h>
#include <math.h>
#include <stdbool.h>

#include "DriverStationLCD.h"
#include "ReversableJaguar.h"
#include "Robot980.h"
#include "Target.h"
#include "numbers.h"

static Robot980* g_pInstance = NULL;

Robot980* Robot980::GetInstance()
{
    if (!g_pInstance)
    {
        g_pInstance = new Robot980();
    }
    return g_pInstance;
}

#define ENC_SCALE	CounterBase::k1X

Robot980::Robot980()
    : m_bTraction(true)

    // left and right drive motors
    , m_pscLeft(new ReversableJaguar(SLOT_PWM_LEFT, CHAN_PWM_LEFT, true))
    , m_pscRight(new Jaguar(SLOT_PWM_RIGHT,CHAN_PWM_RIGHT))
    // lower and upper drive belts
    , m_pscLowerBelt(new Jaguar(SLOT_PWM_LOWER, CHAN_PWM_LOWER))
    , m_pscUpperBelt(new Jaguar(SLOT_PWM_UPPER, CHAN_PWM_UPPER))
    // ball release
    , m_pscFlap(new Jaguar(SLOT_PWM_FLAP, CHAN_PWM_FLAP))

    // sensors
    , m_pGyro(new Gyro(SLOT_GYRO, CHAN_GYRO))

    // encoders on the drive wheels
    , m_pEncDrvLeft(new Encoder(SLOT_ENC_DRV_LEFT,CHAN_ENC_DRV_LEFT_A,
                                SLOT_ENC_DRV_LEFT,CHAN_ENC_DRV_LEFT_B,
                                false, ENC_SCALE))
    , m_pEncDrvRight(new Encoder(SLOT_ENC_DRV_RIGHT,CHAN_ENC_DRV_RIGHT_A,
                                 SLOT_ENC_DRV_RIGHT,CHAN_ENC_DRV_RIGHT_B,
                                 false, ENC_SCALE))

    // encoders on the follow wheels
    , m_pEncFollowLeft(new Encoder(SLOT_ENC_FOLLOW_LEFT,
                                   CHAN_ENC_FOLLOW_LEFT_A,
                                   SLOT_ENC_FOLLOW_LEFT,
                                   CHAN_ENC_FOLLOW_LEFT_B,
                                   false, ENC_SCALE))
    , m_pEncFollowRight(new Encoder(SLOT_ENC_FOLLOW_RIGHT,
                                    CHAN_ENC_FOLLOW_RIGHT_A,
                                    SLOT_ENC_FOLLOW_RIGHT,
                                    CHAN_ENC_FOLLOW_RIGHT_B,
                                    false, ENC_SCALE))

    , m_pSrvPan(new Servo(CAMERA_SLOT_PAN, CAMERA_CHAN_PAN))
    , m_pSrvTilt(new Servo(CAMERA_SLOT_TILT, CAMERA_CHAN_TILT))

    , m_pCamControlLoop(new Notifier(Robot980::CallCamUpdate, this))
    , m_dSavedImageTimestamp(0.0)
    , m_trackColor(DriverStation::kInvalid)
{
    // 48:32 = 1.5 on drive wheels
    m_pEncDrvLeft->SetDistancePerPulse(M_PI * 6 * 1.5 / 250 / 12);
    m_pEncDrvRight->SetDistancePerPulse(M_PI * 6 * 1.5 / 250 / 12);

    m_pEncFollowLeft->SetDistancePerPulse(M_PI * 60/25.4 / 250 / 12);
    m_pEncFollowRight->SetDistancePerPulse(M_PI * 60/25.4 / 250 / 12);

    m_pEncDrvLeft->Start();
    m_pEncDrvRight->Start();
    m_pEncFollowLeft->Start();
    m_pEncFollowRight->Start();

    // "Smart Drive" handles PID, slipping, etc
    m_psdLeft  = new SmartDrive(0.6, 0.1, 0, // velocity PID constants
                                0.1, 0.1, 0, // correction PID constants
                                0.2, 0.1, 0, // acceleration PID constants
                                m_pscLeft, m_pEncDrvLeft, m_pEncFollowLeft);
    m_psdRight = new SmartDrive(0.6, 0.1, 0, // velocity PID constants
                                0.1, 0.1, 0, // correction PID constants
                                0.2, 0.1, 0, // acceleration PID constants
                                m_pscRight, m_pEncDrvRight, m_pEncFollowRight);


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

    printf("Robot980 constructor\n");

    m_trailerInfo.color = DriverStation::kInvalid;

    /* start the CameraTask  */
    StartCameraTask(CAMERA_FPS, CAMERA_COMPRESSION, CAMERA_RESOLUTION,
                    CAMERA_ROTATION);
    m_pVideoServer = new PCVideoServer;
    m_pCamControlLoop->StartPeriodic((double)1.0 / (double)CAMERA_FPS);

    // tell SensorBase about us
    AddToSingletonList();
}

Robot980::~Robot980()
{
    // drive systems (SmartDrive)
    delete m_psdLeft;
    delete m_psdRight;

    // encoders
    delete m_pEncDrvLeft;
    delete m_pEncDrvRight;
    delete m_pEncFollowLeft;
    delete m_pEncFollowRight;

    // speed controllers
    delete m_pscLeft;
    delete m_pscRight;
    delete m_pscLowerBelt;
    delete m_pscUpperBelt;
    delete m_pscFlap;

    // sensors
    delete m_pGyro;

    // camera system
    delete m_pSrvPan;
    delete m_pSrvTilt;

    delete m_pVideoServer;
    delete m_pCamControlLoop;
}

void Robot980::CallCamUpdate(void *pvr)
{
    Robot980* pRobot = static_cast<Robot980*>(pvr);
    pRobot->CamUpdate();
}

void Robot980::EnableTractionControl(bool b)
{
    m_bTraction = b;

    if (m_bTraction)
    {
        m_psdLeft->Enable();
        m_psdRight->Enable();
    }
    else
    {
        m_psdLeft->Disable();
        m_psdRight->Disable();
    }
}

// 1 = forward, -1 = backwards
void Robot980::Drive(float left, float right, DriverStationLCD* pLCD)
{
    if (m_bTraction)
    {
        m_psdLeft->Set(left);
        m_psdRight->Set(right);
    }
    else
    {
        m_pscLeft->Set(left);
        m_pscRight->Set(right);
    }

    Dashboard &d = DriverStation::GetInstance()->GetDashboardPacker();
    if (pLCD)
    {
        static double dPrevEncDrvCntLf = 0;
        static double dPrevEncFlwCntLf = 0;
        static double dPrevEncDrvCntRt = 0;
        static double dPrevEncFlwCntRt = 0;

        double dEncDrvCntLf = m_pEncDrvLeft->GetDistance();
        double dEncFlwCntLf = m_pEncFollowLeft->GetDistance();
        double dEncDrvCntRt = m_pEncDrvRight->GetDistance();
        double dEncFlwCntRt = m_pEncFollowRight->GetDistance();

        double dEncDrvRateLf = dEncDrvCntLf - dPrevEncDrvCntLf;
        double dEncFlwRateLf = dEncFlwCntLf - dPrevEncFlwCntLf;
        double dEncDrvRateRt = dEncDrvCntRt - dPrevEncDrvCntRt;
        double dEncFlwRateRt = dEncFlwCntRt - dPrevEncFlwCntRt;

        dPrevEncDrvCntLf = dEncDrvCntLf;
        dPrevEncFlwCntLf = dEncFlwCntLf;
        dPrevEncDrvCntRt = dEncDrvCntRt;
        dPrevEncFlwCntRt = dEncFlwCntRt;

        dEncDrvRateLf = dEncDrvRateLf / 0.78;
        dEncFlwRateLf = dEncFlwRateLf / 0.78;
        dEncDrvRateRt = dEncDrvRateRt / 0.78;
        dEncFlwRateRt = dEncFlwRateRt / 0.78;

        pLCD->Printf(DriverStationLCD::kMain_Line6, 1,
                     m_bTraction ? "TC: ON " : "TC: OFF");
        pLCD->Printf(DriverStationLCD::kUser_Line2, 1,
                     m_bTraction ? "TC: ON " : "TC: OFF");

        d.Printf(m_bTraction ? "TC: ON \n" : "TC: OFF\n");


        pLCD->Printf(DriverStationLCD::kUser_Line3, 1,
                     "Cmd %1.6f %1.6f", left, right);

        d.Printf("Cmd %1.6f %1.6f\n", left, right);

        pLCD->Printf(DriverStationLCD::kUser_Line4, 1,
                     "Mtr %1.6f %1.6f",
                     m_pscLeft->Get(),
                     m_pscRight->Get());

        d.Printf("Mtr %1.6f %1.6f\n",
               m_pscLeft->Get(),
               m_pscRight->Get());

        pLCD->Printf(DriverStationLCD::kUser_Line5, 1,
                     "Drv %1.6f %1.6f",
                     m_pEncDrvLeft->GetRate(),
                     m_pEncDrvRight->GetRate());

        d.Printf("Drv %1.6f %1.6f\n",
                 dEncDrvRateLf, dEncDrvRateRt);
//               m_pEncDrvLeft->GetRate(),
//               m_pEncDrvRight->GetRate());

        pLCD->Printf(DriverStationLCD::kUser_Line6, 1,
                     "Fol %1.6f %1.6f",
                     m_pEncFollowLeft->GetRate(),
                     m_pEncFollowRight->GetRate());

        d.Printf("Fol %1.6f %1.6f\n",
                 dEncFlwRateLf, dEncFlwRateRt);
//               m_pEncFollowLeft->GetRate(),
//               m_pEncFollowRight->GetRate());

        pLCD->UpdateLCD();
    }
    else
    {
        d.Printf("No pLCD\n");
    }
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

void Robot980::CamUpdate()
{
    // search variables
    bool foundColor = false;

    ParticleAnalysisReport par1, par2; // particle analysis reports
    memset(&par1, 0, sizeof(ParticleAnalysisReport));
    memset(&par2, 0, sizeof(ParticleAnalysisReport));

    // calculate gimbal position based on colors found
    if (FindTwoColors(m_tdPink, m_tdGreen,
                      (m_trackColor == DriverStation::kBlue) ? ABOVE : BELOW,
                      &par1, &par2))
    {
        foundColor = true;

        if (par1.imageTimestamp == m_dSavedImageTimestamp)
        {
            // This image has been processed already, so don't do anything
            return;
        }

        // The target was recognized - save the timestamp
        m_dSavedImageTimestamp = par1.imageTimestamp;

        m_trailerInfo.color = m_trackColor;

        // get center of target; Average the color two particles to
        // get center x & y of combined target
        m_trailerInfo.dCenterMassX = (par1.center_mass_x_normalized +
                                      par2.center_mass_x_normalized) / 2;
        m_trailerInfo.dCenterMassY = (par1.center_mass_y_normalized +
                                      par2.center_mass_y_normalized) / 2;

        m_trailerInfo.particleQuality = (par1.particleQuality +
                                         par2.particleQuality) / 2;


    }
    else
    {                   // need to pan
        foundColor = false;
    }

    if (foundColor)
    {
        /* Move the servo a bit each loop toward the destination.
         * Alternative ways to task servos are to move immediately vs.
         * incrementally toward the final destination. Incremental method
         * reduces the need for calibration of the servo movement while
         * moving toward the target.
         */
    }
    else
    {
        // new image, but didn't find two colors


    } // end if found color
}

bool Robot980::FindTrailer(DriverStation::Alliance alliance,
                           trailerInfo_t *pTrailer)
{
    // Are we switching which color we're looking for?
    if (alliance != m_trackColor)
    {
        m_trackColor = alliance;
        return false;
    }

    return false;
}
