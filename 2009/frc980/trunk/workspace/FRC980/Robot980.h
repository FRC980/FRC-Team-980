#ifndef ROBOT980_H
#define ROBOT980_H

// The slots for the Digital Side Car installed on the left & right side
// of the robot
#define DSC_LEFT                    6
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
#define CHAN_PWM_RIGHT              1

#define SLOT_PWM_LOWER              DSC_RIGHT
#define CHAN_PWM_LOWER              2

#define SLOT_PWM_UPPER              DSC_LEFT
#define CHAN_PWM_UPPER              2

#define SLOT_PWM_FLAP               DSC_LEFT
#define CHAN_PWM_FLAP               3

// Digital Inputs
#define SLOT_ENC_DRV_LEFT           DSC_LEFT
#define CHAN_ENC_DRV_LEFT_A         1
#define CHAN_ENC_DRV_LEFT_B         2

#define SLOT_ENC_FOLLOW_LEFT        DSC_LEFT
#define CHAN_ENC_FOLLOW_LEFT_A      3
#define CHAN_ENC_FOLLOW_LEFT_B      4

#define SLOT_ENC_DRV_RIGHT          DSC_RIGHT
#define CHAN_ENC_DRV_RIGHT_A        1
#define CHAN_ENC_DRV_RIGHT_B        2

#define SLOT_ENC_FOLLOW_RIGHT       DSC_RIGHT
#define CHAN_ENC_FOLLOW_RIGHT_A     3
#define CHAN_ENC_FOLLOW_RIGHT_B     4

// Analog Inputs
#define SLOT_GYRO                   1
#define CHAN_GYRO                   1 /* FPGA requires gyro to be on chan 1 */


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

class Robot980 : public SensorBase
{
  public:
    static Robot980 *GetInstance();

    bool FindTrailer(DriverStation::Alliance); // will have more params
    void EnableTractionControl(bool);

    // 1 = forward, -1 = backwards
    void Drive(float left, float right, DriverStationLCD* pLCD = NULL);

    void RunBelts(float lower, float upper); // 1 = in @ base, up and out
    void Flap(bool open);
    bool GetTractionControl() { return m_bTraction; };

    float getAngle();           // get angle from gyro
    void SetTrackColor(DriverStation::Alliance);

  private:
    Robot980();
    virtual ~Robot980();

    bool m_bTraction;

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

};

#endif // ROBOT980_H
