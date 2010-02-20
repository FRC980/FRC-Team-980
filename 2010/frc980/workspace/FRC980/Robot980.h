#ifndef ROBOT980_H
#define ROBOT980_H

#include "numbers.h"

//==============================================================================
/*! The Gear Ratio from output of gearbox to wheel */
#define GEAR_RATIO      ((double)1)

/*! The Gearbox Ratio ~ 8.4586:1 */
#define GEARBOX_RATIO   ((double)50/(double)14*(double)45/(double)19)

/*! The Roller Motor Gearbox Ratio ~ 3.67:1 */
#define ROLLER_GEARBOX  ((double)11/(double)3)

/*! Theoretical speed of CIM is 5500 RPM
 *  free-running top speed (in ft/sec) = (speed of cim in rpm) / 60sec/min
 *  (ratio of toughbox) * (sprocket ratio) * pi * (wheel diameter in feet)
 */
const double TOP_SPEED = ((double)5500/(double)60 / (GEARBOX_RATIO) * (GEAR_RATIO) * M_PI * (double)0.5); /* ~ 17 ft/sec */

//==============================================================================
// CAN Jaguar Outputs

#define CAN_LEFT_CIM                11   /*!< \def CAN_LEFT_CIM The CAN Jaguar device number for the Left CIM Motor */
#define CAN_LEFT_FP                 12   /*!< \def CAN_LEFT_FP The CAN Jaguar device number for the Left Fisher Price Motor */
#define CAN_RIGHT_CIM               13   /*!< \def CAN_RIGHT_CIM The CAN Jaguar device number for the Right CIM Motor */
#define CAN_RIGHT_FP                14   /*!< \def CAN_RIGHT_FP The CAN Jaguar device number for the Right Fisher Price Motor */
#define CAN_ROLLER                  15   /*!< \def CAN_ROLLER The CAN Jaguar device number for the Roller Motor */
#define CAN_WINCH                   16   /*!< \def CAN_WINCH The CAN Jaguar device number for the Winch Motor */

//==============================================================================
// Digital Side Car Outputs

// Digital Side Car Slot
#define DSC_SLOT                    4   /*!< \def DSC_SLOT The slot number in the cRio for the Digital Side Car */

// PWM outputs
#define CHAN_PWM_ARM1               1   /*!< \def CHAN_PWM_ARM1 The Digital Side Car PWM Channel for the Kick Arming Motor number 1*/
#define CHAN_PWM_ARM2               2   /*!< \def CHAN_PWM_ARM2 The Digital Side Car PWM Channel for the Kick Arming Motor number 2*/
#define CHAN_PWM_FIRE               3   /*!< \def CHAN_PWM_FIRE The Digital Side Car PWM Channel for the Kick Firing Motor*/

// Camera Servos
#define CAMERA_CHAN_PAN             7   /*!< \def CAMERA_CHAN_PAN The Digital Side Car PWM Channel for the Camera Pan Servo */
#define CAMERA_CHAN_TILT            8   /*!< \def CAMERA_CHAN_TILT The Digital Side Car PWM Channel for the Camera Tilt Servo */

//==============================================================================
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

//==============================================================================
// Analog Inputs
#define SLOT_GYRO                   1   /*!< SLOT_GYRO The slot number in the cRio for the Gyro analog input*/
#define CHAN_GYRO                   1   /*!< CHAN_GYRO The Analog Channel for the Gyro. FPGA requires gyro to be on chan 1 */

#define SLOT_AUTO_MODE              1   /*!< SLOT_AUTO_MODE The slot number in the cRio for the Auto Mode analog input*/
#define CHAN_AUTO_MODE              7   /*!< CHAN_AUTO_MODE The Analog Channel for the Auto Mode. */

//==============================================================================
// Define Additional Values

#ifndef NULL
#define NULL                        (0) /*!< \def NULL The NULL value is created here because it is not created elsewhere */
#endif // NULL

//==============================================================================
class Encoder;
class Gyro;
class PCVideoServer;
class SpeedController;
class Timer;

//==============================================================================
//! Code specific to the Team 980 Robot
/*!\class Robot980
 *
 * The purpose of the Robot980 class is to contain code specific to the team's
 * robot.
 *    
 */
class Robot980 : public SensorBase
{
   private:
      //--- Instance Variables -------------------------------------------------
      //--- Jaguars
      
      // left and right drive motors
      SpeedController* m_pscLeft_cim;     /*!< The Left CIM motor speed controller */
      SpeedController* m_pscLeft_fp;      /*!< The Left FP motor speed controller */
      SpeedController* m_pscRight_cim;    /*!< The Right CIM motor speed controller */
      SpeedController* m_pscRight_fp;     /*!< The Right FP motor speed controller */
      
      // roller and winch motors
      SpeedController* m_pscRoller;       /*!< The Roller motor speed controller */
      SpeedController* m_pscWinch;        /*!< The Winch motor speed controller */
      
      //--- Victors
      SpeedController* m_pscArm1;         /*!< The Arming motor 1 speed controller */
      SpeedController* m_pscArm2;         /*!< The Arming motor 2 speed controller */
      SpeedController* m_pscFire;         /*!< The Firing motor speed controller */
      
      //--- Encoders on the drive wheels
      Encoder* m_pEncDrvLeft;             /*!< The Left motor drive encoder */
      Encoder* m_pEncDrvRight;            /*!< The Right motor drive encoder */
      Encoder* m_pEncRoller;              /*!< The Roller motor drive encoder */
      
      //--- Sensors
      Gyro* m_pGyro;                      /*!< The Gyro Sensor */
      // more sensors TBD
      
      //--- Timers
      Timer* m_pTimerDrive;              /*!< The Timer used for debugging (calc & print speeds) */
      Timer* m_pTimerFire;               /*!< The Timer used for firing. Can only fire once every 2 seconds */
      
      //--- Camera
      Servo* m_pSrvPan;                  /*!< The Camera Pan Servo */
      Servo* m_pSrvTilt;                 /*!< The Camera Tilt Servo */
      PCVideoServer* m_pVideoServer;     /*!< The Camera Video Source */
      
      //--- Constructors -------------------------------------------------------
      /*! \brief The Robot 980 Constructor
       *
       *  The constructor/destructor are private to enforce this being a
       *  singleton object, accessible only via Robot980::GetInstance();
       */
      Robot980();
      
      //--- Destructors --------------------------------------------------------
      /*! \brief The Robot 980 Destructor
       *
       *  The constructor/destructor are private to enforce this being a
       *  singleton object, accessible only via Robot980::GetInstance();
       */
      virtual ~Robot980();
    
   public:
      //--- Instance Variables -------------------------------------------------
      
      //--- Constructors -------------------------------------------------------
      static Robot980 *GetInstance();
      
      //--- Methods ------------------------------------------------------------
      /*!\brief The accessor method for the autonomous mode
       *
       *  There will be a multi-position switch wired as a potentiometer,
       *  connected to a single analog input.  GetAutonMode reads that analog
       *  value and converts it to a single "mode" integer.
       */
      int GetAutonMode(void);
      
      /*! \brief A drive method to move the robot and control the roller
       *  \param left The speed of the left drive motors
       *  \param right The speed of the right drive motors
       *  \param roller The speed of the roller motor
       *  
       *  This method is used to drive the robot and also to control
       *  the speed of the roller.  The roller is independent of the speed of
       *  the drive system in this instance.
       *
       *  For reference: 1 = forward, -1 = backwards
       */
      void Drive(float left, float right, float roller);
      
      /*! \brief A drive method to move the robot and autonomously control the roller
       *  \param left The speed of the left drive motors
       *  \param right The speed of the right drive motors
       *  
       *  This method is used to drive the robot and also to control
       *  the speed of the roller.  The roller is dependent on the speed of
       *  the drive system in this instance.
       *
       *  For reference: 1 = forward, -1 = backwards
       */
      void Drive(float left, float right);
      
      /*! \brief A method to run the robot lift motor
       *  \param speed The speed to set the lift motor
       *  \todo Write this method
       */
      void Lift(float speed);
      
      /*! \brief Determine if the kicker has been retracted
       *  \return true if the kicker is retracted (and the time restriction
       *  has passed)
       *  
       *  This method is used to determine if the kicker has been retracted.
       *
       *  \todo finish this method
       */
      bool KickerArmed(void);
      
      /*! \brief Arm the kicker
       *  
       *  This method is used to arm the kicking mechanism by retracting
       *  the kicker
       *
       *  \todo finish this method
       */
      void ArmKicker(void);
      
      /*! \brief Determine if the kicker has been fired
       *  \return true if the kicker has fired
       *  
       *  This method is used to determine if the kicker has been retracted.
       *
       *  \todo finish this method
       */
      bool KickerFired(void);
      
      /*! \brief Fire the kicker
       *  
       *  This method is used to Fire the kicking mechanism
       *
       *  \todo finish this method
       */
      void FireKicker(void);
      
      /*! \brief Get the angle from the gyro
       *
       *  This method is used to get the angle information from the
       *  gyro onboard the robot
       *  
       *  \todo Positioning system - gyro, accelerometer
       */
      float GetAngle(void);
      
      /*! \brief Determine if the Target is available
       *  \return true if ball goal is in front of robot's field of view
       *
       *  \todo Complete Camera system, target and tracking
       */
      bool TargetAvailable(void);
};

#endif // ROBOT980_H
