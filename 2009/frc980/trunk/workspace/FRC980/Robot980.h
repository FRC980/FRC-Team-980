#ifndef ROBOT980_H
#define ROBOT980_H

// practice bot
#define GEAR_RATIO                  54/32
#define TOP_SPEED                   18

// competition bot
//#define GEAR_RATIO                  48/36
//#define TOP_SPEED                   12 /* VERIFY */

// The slots for the Digital Side Car installed on the left & right side
// of the robot
#define DSC_SLOT                    4
#define DSC_LEFT                    4
#define DSC_RIGHT                   4

// Camera
#define CAMERA_FPS                  20
#define CAMERA_COMPRESSION          0
//#define CAMERA_RESOLUTION           k160x120
#define CAMERA_RESOLUTION           k320x240
#define CAMERA_ROTATION             ROT_180

#define CAMERA_SLOT_PAN             DSC_RIGHT
#define CAMERA_CHAN_PAN             4
#define CAMERA_SLOT_TILT            DSC_RIGHT
#define CAMERA_CHAN_TILT            5


// PWM outputs
#define SLOT_PWM_LEFT               DSC_LEFT
#define CHAN_PWM_LEFT               1

#define SLOT_PWM_RIGHT              DSC_RIGHT
#define CHAN_PWM_RIGHT              2

#define SLOT_PWM_LOWER              DSC_RIGHT
#define CHAN_PWM_LOWER              3

#define SLOT_PWM_UPPER              DSC_LEFT
#define CHAN_PWM_UPPER              4

#define SLOT_PWM_FLAP               DSC_LEFT
#define CHAN_PWM_FLAP               5

// Digital Inputs
#define SLOT_ENC_DRV_LEFT           DSC_LEFT
#define CHAN_ENC_DRV_LEFT_A         1
#define CHAN_ENC_DRV_LEFT_B         2

#define SLOT_ENC_FOLLOW_LEFT        DSC_LEFT
#define CHAN_ENC_FOLLOW_LEFT_A      4
#define CHAN_ENC_FOLLOW_LEFT_B      3

#define SLOT_ENC_DRV_RIGHT          DSC_RIGHT
#define CHAN_ENC_DRV_RIGHT_A        6
#define CHAN_ENC_DRV_RIGHT_B        5

#define SLOT_ENC_FOLLOW_RIGHT       DSC_RIGHT
#define CHAN_ENC_FOLLOW_RIGHT_A     8
#define CHAN_ENC_FOLLOW_RIGHT_B     7

// Analog Inputs
#define SLOT_GYRO                   1
#define CHAN_GYRO                   1 /* FPGA requires gyro to be on chan 1 */

#define SLOT_AUTO_MODE              1
#define CHAN_AUTO_MODE              7



#ifndef NULL
#define NULL    (0)
#endif // NULL

#include "SmartDrive.h"
#include <DriverStation.h>
#include <TrackAPI.h>

class DriverStationLCD;
class Encoder;
class Gyro;
class PCVideoServer;
class SpeedController;
class Timer;

class Robot980 : public SensorBase
{
  public:
    typedef enum
    {
        TC_OFF,
        TC_LOWPASS,
        TC_SMART,
    } tractionMode_t;

    typedef struct
    {
        DriverStation::Alliance color; // kInvalid indicates none found
        double dDistance;

        double dHorizLocation;

        // target's center of mass within camera view, normalized -1 to +1
        double dCenterMassX;
        double dCenterMassY;

        /* Area of the particle */
        double particleArea;
        double particleToImagePercent;

        double particleQuality;




        // Where is the camera pointing?  Adjusted such that -1 = left &
        // down; 1 = right & up
        float fPan;
        float fTilt;
    } trailerInfo_t;

  public:
    static Robot980 *GetInstance();

    int GetAutonMode();

    bool FindTrailer(DriverStation::Alliance trackColor,
                     trailerInfo_t *pTrailer);
    void EnableTractionControl(tractionMode_t);

    // 1 = forward, -1 = backwards
    void Drive(float left, float right, DriverStationLCD* pLCD = NULL);

    void RunBelts(float lower, float upper); // 1 = in @ base, up and out
    void Flap(bool open);
    tractionMode_t GetTractionControl() { return m_tcTraction; };

    float getAngle();           // get angle from gyro
    void SetTrackColor(DriverStation::Alliance);

  private:
    Robot980();
    virtual ~Robot980();

    tractionMode_t m_tcTraction;

    // left and right drive systems
    SmartDrive* m_psdLeft;
    SmartDrive* m_psdRight;

    // left and right drive motors
    SpeedController* m_pscLeft;
    SpeedController* m_pscRight;

    // lower and upper drive belts
    SpeedController* m_pscLowerBelt;
    SpeedController* m_pscUpperBelt;

    // ball release
    SpeedController* m_pscFlap;

    // sensors
    Gyro* m_pGyro;

    // encoders on the drive wheels
    Encoder* m_pEncDrvLeft;
    Encoder* m_pEncDrvRight;

    // encoders on the follow wheels
    Encoder* m_pEncFollowLeft;
    Encoder* m_pEncFollowRight;

    // timer used for debugging (calculating & printing speeds)
    Timer* m_pTimer;

    // camera pan/tilt servos
    Servo* m_pSrvPan;
    Servo* m_pSrvTilt;

    PCVideoServer* m_pVideoServer;
    TrackingThreshold m_tdPink, m_tdGreen; // color thresholds
    Notifier* m_pCamControlLoop;
    static void CallCamUpdate(void *pvr);
    void CamUpdate();
    double m_dSavedImageTimestamp;
    DriverStation::Alliance m_trackColor;

    trailerInfo_t m_trailerInfo;
};

#endif // ROBOT980_H
