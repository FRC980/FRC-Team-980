#ifndef ROBOT980_H
#define ROBOT980_H

// PWM outputs
#define SLOT_PWM_LEFT               4
#define CHAN_PWM_LEFT               1

#define SLOT_PWM_RIGHT              4
#define CHAN_PWM_RIGHT              2

#define SLOT_PWM_LOWER              4
#define CHAN_PWM_LOWER              3

#define SLOT_PWM_UPPER              4
#define CHAN_PWM_UPPER              4

#define SLOT_PWM_FLAP               4
#define CHAN_PWM_FLAP               5

// Digital Inputs
#define SLOT_ENC_DRV_LEFT           4
#define CHAN_ENC_DRV_LEFT_A         1
#define CHAN_ENC_DRV_LEFT_B         2

#define SLOT_ENC_FOLLOW_LEFT        4
#define CHAN_ENC_FOLLOW_LEFT_A      3
#define CHAN_ENC_FOLLOW_LEFT_B      4

#define SLOT_ENC_DRV_RIGHT          4
#define CHAN_ENC_DRV_RIGHT_A        5
#define CHAN_ENC_DRV_RIGHT_B        6

#define SLOT_ENC_FOLLOW_RIGHT       4
#define CHAN_ENC_FOLLOW_RIGHT_A     7
#define CHAN_ENC_FOLLOW_RIGHT_B     8

// Analog Inputs
#define SLOT_GYRO                   1
#define CHAN_GYRO                   1 /* FPGA requires gyro to be on chan 1 */


#ifndef NULL
#define NULL	(0)
#endif // NULL

#include "SmartDrive.h"

class Encoder;
class Gyro;
class SpeedController;

class Robot980
{
  public:
    static Robot980 *GetInstance();

    void Drive(float left, float right); // 1 = forward, -1 = backwards
    void RunBelts(float lower, float upper); // 1 = in @ base, up and out
    void Flap(bool open);

    float getAngle();           // get angle from gyro


  private:
    Robot980();
    virtual ~Robot980();

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

    // encoders on the rotation of the follow wheels (casters)
    Encoder* m_pEncRotateLeft;
    Encoder* m_pEncRotateRight;
};

#endif // ROBOT980_H
