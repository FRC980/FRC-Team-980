#ifndef ROBOT980_H
#define ROBOT980_H

#include "numbers.h"

// competition bot
#define GEAR_RATIO                  (1)

// Theoretical speed of CIM is 5500 RPM
// free-running top speed = <speed of cim in rpm> / 60sec/min / <ratio of
// toughbox> * <sprocket ratio> * pi * <wheel diameter in feet>
#define TOP_SPEED   ((double)5500/(double)60 / (double)12.75 * (GEAR_RATIO) * M_PI * (double)0.5)

// The slots for the Digital Side Car installed on the left & right side
// of the robot
#define DSC_SLOT                    4

// PWM outputs
#define CHAN_PWM_LEFT               1
#define CHAN_PWM_RIGHT              2
#define CHAN_PWM_ROLLER             3
#define CHAN_PWM_ARMER              4
#define CHAN_PWM_FIRE               5
#define CHAN_PWM_LIFT               6

#define CAMERA_CHAN_PAN			    7
#define CAMERA_CHAN_TILT            8

// Digital Inputs
#define CHAN_ENC_DRV_LEFT_A         1
#define CHAN_ENC_DRV_LEFT_B         2

#define CHAN_ENC_DRV_RIGHT_A        3
#define CHAN_ENC_DRV_RIGHT_B        4

#define CHAN_ENC_ROLLER_A           5
#define CHAN_ENC_ROLLER_B           6

#define CHAN_ENC_ARMER_A            7
#define CHAN_ENC_ARMER_B            8

#define CHAN_ENC_ARMER_A            7
#define CHAN_ENC_ARMER_B            8

#define CHAN_ENC_FIRE_A             9
#define CHAN_ENC_FIRE_B             10

#define CHAN_ENC_LIFT_A             11
#define CHAN_ENC_LIFT_B             12

// Analog Inputs
#define SLOT_GYRO                   1
#define CHAN_GYRO                   1 /* FPGA requires gyro to be on chan 1 */

#define SLOT_AUTO_MODE              1
#define CHAN_AUTO_MODE              7


#ifndef NULL
#define NULL    (0)
#endif // NULL

class Encoder;
class Gyro;
class PCVideoServer;
class SpeedController;
class Timer;

class Robot980 : public SensorBase
{
  public:

  public:
    static Robot980 *GetInstance();

    int GetAutonMode();

    // 1 = forward, -1 = backwards
	void Drive(float left, float right, float roller);
	
	// automatically determines roller speed if unspecified
	void Drive(float left, float right);
	
	//TODO: Lift system
	
	//If the kicker is not fully retracted, retract it some more
	//Returns true if the kicker is retracted [and the time restriction has passed]
	bool KickerArm();
	
	void Kick();
	
	//TODO: Positioning system - gyro, accelerometer
	
	//TODO: Camera system and target tracking

    float getAngle();           // get angle from gyro

  private:
    Robot980();
    virtual ~Robot980();

    // left and right drive motors
    SpeedController* m_pscLeft;
    SpeedController* m_pscRight;

    SpeedController* m_pscRoller;
    SpeedController* m_pscArmer;
    SpeedController* m_pscFire;

    SpeedController* m_pscLift;

    // sensors
    Gyro* m_pGyro;

    // encoders on the drive wheels
    Encoder* m_pEncDrvLeft;
    Encoder* m_pEncDrvRight;

    Encoder* m_pEncRoller;

    // more sensors TBD

    // timer used for debugging (calculating & printing speeds)
    Timer* m_pTimer;

    // camera pan/tilt servos
    Servo* m_pSrvPan;
    Servo* m_pSrvTilt;

    PCVideoServer* m_pVideoServer;
};

#endif // ROBOT980_H
