#ifndef ROBOT980_H
#define ROBOT980_H

#include "numbers.h"

/* ratio from output of gearbox to wheel */
#define GEAR_RATIO      ((double)1)

/* ~ 8.4586:1 */
#define GEARBOX_RATIO   ((double)50/(double)14*(double)45/(double)19)

/* ~ 3.67:1 */
#define ROLLER_GEARBOX  ((double)11/(double)3)

// Theoretical speed of CIM is 5500 RPM
// free-running top speed (in ft/sec) = <speed of cim in rpm> / 60sec/min
//   / <ratio of toughbox> * <sprocket ratio> * pi * <wheel diameter in feet>
const double TOP_SPEED = ((double)5500/(double)60 / (GEARBOX_RATIO) * (GEAR_RATIO) * M_PI * (double)0.5); /* ~ 17 ft/sec */

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

#define CAMERA_CHAN_PAN             7
#define CAMERA_CHAN_TILT            8

// Digital Inputs

// Encoders on left and right drive wheels
#define CHAN_ENC_DRV_LEFT_A         1
#define CHAN_ENC_DRV_LEFT_B         2

#define CHAN_ENC_DRV_RIGHT_A        3
#define CHAN_ENC_DRV_RIGHT_B        4

// Encoder on the roller, used to hold the ball
#define CHAN_ENC_ROLLER_A           5
#define CHAN_ENC_ROLLER_B           6

// Encoder on the arming mechanism -- there may also be a limit switch to
// indicate the mechanism is armed.
#define CHAN_ENC_ARMER_A            7
#define CHAN_ENC_ARMER_B            8

// Encoder on the firing wheel -- this may instead be a potentiometer or 2
// limit switches.
#define CHAN_ENC_FIRE_A             9
#define CHAN_ENC_FIRE_B             10

// Encoder on the lift mechanism
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
    static Robot980 *GetInstance();
    
    // There will be a multi-position switch wired as a potentiometer,
    // connected to a single analog input.  GetAutonMode reads that analog
    // value and converts it to a single "mode" integer.
    int GetAutonMode();
    
    // 1 = forward, -1 = backwards
    void Drive(float left, float right, float roller);
    
    // automatically determines roller speed if unspecified
    void Drive(float left, float right);
    
    //TODO: Lift system
    
    // Returns true if the kicker is retracted [and the time restriction
    // has passed]
    bool CanKick();
    
    // Fire the kicker, and then automatically re-arm
    bool Kick();
    
    // TODO: Positioning system - gyro, accelerometer
    float getAngle();           // get angle from gyro
    
    // TODO: Camera system and target tracking
    
  private:
    // constructor/destructor are private to enforce this being a
    // singleton object, accessible only via Robot980::GetInstance();
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
    
    Timer* m_pTimerDrive; // timer used for debugging (calc & print speeds)
    Timer* m_pTimerFire;  // can only fire once every 2 seconds
    
    // camera pan/tilt servos
    Servo* m_pSrvPan;
    Servo* m_pSrvTilt;
    
    PCVideoServer* m_pVideoServer;
};

#endif // ROBOT980_H
