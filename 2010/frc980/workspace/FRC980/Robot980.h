#ifndef ROBOT980_H
#define ROBOT980_H

//--- Include the appropriate libraries
#include "numbers.h" // This includes numerical constants used by the robot

#include "CANJaguar.h" // This includes the necessary methods to use the CAN Jaguar
#include <Vision/PCVideoServer.h>

// REVIEW: Style -- include full header, or just use a forward class
// declaration?  Most of these get pulled in from "CANJaguar.h".

// class CANJaguar;
// class DigitalInput;
// class Notifier;
// class PCVideoServer;
// class Timer;
// class Victor;

//==============================================================================
/*
 * FRC Team 980 - Wiring Diagram
 *
 * Jaguars
 * =======
 *
 * Left Drive CIM 1  --> Jag11
 * Left Drive CIM 2  --> Jag12
 * Right Drive CIM 1 --> Jag13
 * Right Drive CIM 2 --> Jag14
 * Lift Motor        --> Jag15
 * Roller FP         --> Jag16
 *
 *
 * Victors
 * =======
 *
 * Arming Window Motor 1 --> Victor1
 * Arming Window Motor 2 --> Victor2
 * Fire Window Motor     --> Victor3
 *
 *
 * Digital Side Car
 * ================
 *
 * PWM Channels
 * ------------
 * Arming Motors 1/2  --> DSC D/O 1
 * Fire Motors        --> DSC D/O 2
 *
 * Limit Switches
 * --------------
 * Arming Switch (detects kicker in armed position) --> DSC D/I 1
 * Fire Cam Switch (detects firing cam in position) --> DSC D/I 2
 * Winch Switch (counts winch rotations)            --> DSC D/I 3
 *
 *
 * Classmate PC
 * ============
 * Login: Driver; NO PASSWORD
 * Login: Developer; NO PASSWORD
 *
 * Classmate Ethernet Port    --> Wireless Hub
 * 4-Port USB Expansion Black --> Left Classmate USB
 * Red   --> Right Classmate USB (This is optional)
 *
 * External Accessories
 * ====================
 *
 * Joystick 1  --> USB Expansion Port
 * Joystick 2  --> USB Expansion Port
 * Stop Button --> USB Expansion Port
 *
 */

//==============================================================================
/*! \def GEAR_RATIO
 * The Gear Ratio from output of gearbox to wheel
 */
#define GEAR_RATIO      ((double)1)

/*! \def GEARBOX_RATIO
 * The Gearbox Ratio ~ 8.4586:1
 */
#define GEARBOX_RATIO   ((double)50/(double)14*(double)45/(double)19)

/*! \def ROLLER_GEARBOX
 * The Roller Motor Gearbox Ratio ~ 3.67:1
 */
#define ROLLER_GEARBOX  ((double)11/(double)3)

/*! \def TOP_SPEED
 *  Theoretical speed of CIM is 5500 RPM
 *  free-running top speed (in ft/sec) = (speed of cim in rpm) / 60sec/min
 *  (ratio of toughbox) * (sprocket ratio) * pi * (wheel diameter in feet)
 */
const double TOP_SPEED = ((double)5500 / (double)60 / (GEARBOX_RATIO) * (GEAR_RATIO) * M_PI * (double)0.5);     /* ~ 17 ft/sec */

//==============================================================================
// CAN Jaguar Outputs
#define CAN_LEFT_CIM1               11  /*!< \def CAN_LEFT_CIM1 The CAN Jaguar device number for the Left CIM Motor */
#define CAN_LEFT_CIM2               12  /*!< \def CAN_LEFT_CIM2 The CAN Jaguar device number for the Left CIM Motor */
#define CAN_RIGHT_CIM1              13  /*!< \def CAN_RIGHT_CIM1 The CAN Jaguar device number for the Right CIM Motor */
#define CAN_RIGHT_CIM2              14  /*!< \def CAN_RIGHT_CIM2 The CAN Jaguar device number for the Right CIM Motor */
#define CAN_LIFT                    15  /*!< \def CAN_LIFT The CAN Jaguar device number for the Lift Motor */
#define CAN_ROLLER_FP               16  /*!< \def CAN_ROLLER_FP The CAN Jaguar device number for the Roller Motor */

// Jaguar Outputs
#define MAX_JAGUAR_OUTPUT_VOLTAGE   12.0        /*!< \def MAX_JAGUAR_OUTPUT_VOLTAGE The maximum output voltage of the CAN Jaguar */
// Encoder setup information
#define US_DIGITAL_ENC_COUNTS       250 /*!< \def US_DIGITAL_ENC_COUNTS The number of encoder counts for the US Digital Encoders */

//==============================================================================
// Digital Side Car Outputs

// Digital Side Car Slot
#define DSC_SLOT                    4   /*!< \def DSC_SLOT The slot number in the cRio for the Digital Side Car */

// PWM outputs
#define CHAN_PWM_ARM                1   /*!< \def CHAN_PWM_ARM The Digital Side Car PWM Channel for both of the Kicker Arming Motors */
#define CHAN_PWM_FIRE               2   /*!< \def CHAN_PWM_FIRE The Digital Side Car PWM Channel for the Kicker Firing Motor */

//==============================================================================
// Digital Inputs

// Limit Switches for Kicking Mechanism
#define CHAN_LIMIT_ARMED            2   /*!< \def CHAN_LIMIT_ARMED The Kicker Arming Limit Switch */
#define CHAN_LIMIT_FIRE_READY       1   /*!< \def CHAN_LIMIT_FIRE_READY The Kicker Firing Limit Switch */
#define CHAN_LIMIT_WINCH_COUNTER    3   /*!< \def CHAN_LIMIT_WINCH The Kicker Winch Limit Switch */

// Encoder on the lift mechanism
//#define CHAN_ENC_LIFT_A             11
//#define CHAN_ENC_LIFT_B             12

//==============================================================================
// Analog Inputs
//#define SLOT_GYRO                   1   /*!< SLOT_GYRO The slot number in the cRio for the Gyro analog input*/
//#define CHAN_GYRO                   1   /*!< CHAN_GYRO The Analog Channel for the Gyro. FPGA requires gyro to be on chan 1 */

#define SLOT_AUTO_MODE              1   /*!< SLOT_AUTO_MODE The slot number in the cRio for the Auto Mode analog input */
#define CHAN_AUTO_MODE              7   /*!< CHAN_AUTO_MODE The Analog Channel for the Auto Mode. */

//==============================================================================
// Define Additional Values

//--- Reverse Drive Direction
/*! \def REVERSE_DRIVE This variable is used to reverse the drive signal to
 *  motors that need to be moved in reverse
 */
#define REVERSE_DRIVE               -1.0

//--- Kicker Reset Period
#define KICKER_RESET_PERIOD         2.0 /*!< \def KICKER_RESET_PERIOD The number of seconds that must elapse before the kicker can reset */

// all switches should be wired the same -- either normal open = high, or
// normal open = low.  In our case, normal open = low(?)
#define SW_OPEN     true
#define SW_CLOSED   false

//--- NULL Value
#ifndef NULL
#define NULL                        (0) /*!< \def NULL The NULL value is created here because it is not created elsewhere */
#endif  // NULL

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
    CANJaguar * m_pscLeft_cim1; /*!< The Left CIM1 motor speed controller */
    CANJaguar *m_pscLeft_cim2;  /*!< The Left CIM2 motor speed controller */
    CANJaguar *m_pscRight_cim1; /*!< The Right CIM1 motor speed controller */
    CANJaguar *m_pscRight_cim2; /*!< The Right CIM2 motor speed controller */

    // roller and lift motors
    CANJaguar *m_pscRoller_fp;  /*!< The Roller motor speed controller */
    // CANJaguar* m_pscLift;       /*!< The Lift motor speed controller */

    //--- Victors
    Victor *m_pscArm_win;      /*!< The Arming motor 1 speed controller */
    Victor *m_pscFire_win;     /*!< The Firing motor speed controller */

    //--- Sensors
    //Gyro* m_pGyro;                 /*!< The Gyro Sensor */
    DigitalInput *m_pdiArmed_switch; /*!< The Arming Mechanism Limit Switch */
    DigitalInput *m_pdiFireCam_switch; /*!< The Firing Mechanism Limit Switch */
    DigitalInput *m_pdiWinch_switch; /*!< The Winch Mechanism Limit Switch */
    // more sensors TBD

    //--- Timers
    Timer *m_pTimerDrive;       /*!< The Timer used for debugging (calc & print speeds) */
    Timer *m_pTimerFire;        /*!< The Timer used for firing. Can only fire once every 2 seconds */

    Timer *m_pTimerUnwind;      /*!< Timer used for unwinding */
    Notifier *m_pnWinchPolling; /*!< Notifier class (for periodic asynchronous funcion calls) to poll the winch switches */

    //--- Firing mechanism state
    typedef enum
    {
        UNKNOWN,                /* 0 */
        READY_TO_FIRE,          /* 1 */
        WINDING,                /* 2 */
        START_UNWINDING,        /* 3 */
        UNWINDING,              /* 4 */
    } arming_t;                 /*!< State machine for firing mechanism */
    arming_t m_armingState;     /*!< Current state of firing mechanism */
    bool m_bArmingEnable;       /*!< DEBUG: enable/disable arming */

    PCVideoServer* m_pVideoServer;

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
    virtual ~ Robot980();

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
    int  GetAutonMode(void);

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

    /*! \brief Determine if the kicker has been armed
     *  \return true if the kicker is armed and ready to fire
     *
     *  This method is used to determine if the kicker is armed.
     *  It checks if the kicker has been retracted, the winch unwound,
     *  and the time restriction has passed.
     */
    bool KickerArmed(void);

    /*! \brief Arm the kicker
     *
     *  This method is used to arm the kicking mechanism by retracting
     *  the kicker and then unwinding the winch
     */
    void ArmKicker(void);

    /*! \brief Fire the kicker
     *
     *  This method is used to Fire the kicking mechanism
     */
    void FireKicker(void);

    /*! \brief DEBUG: enable/disable arming
     */
    void ArmingEnable(void);
    
    /*! \brief
     *
     */
    void ArmingDisable(void);

    /*! \brief DEBUG: set constant winch speed
     *  \param speed
     */
    void SetWinch(float speed);
    
    /*! \brief
     *  \todo This should be moved to interrupts on the dig-in switches
     *
     */
    void HandleFiring(void);
    
    /*! \brief
     *  \todo This should be moved to interrupts on the dig-in switches
     *
     */
    void HandleArming(void);

    /*! \brief Handle all "automatic" functions of the robot -- this is NOT autonomous mode
     *
     * This method handles all automatic functions of the robot, such as
     * turning the compressor on or off.
     *
     * In 2010, we don't actually have a compressor, but we do have a
     * firing mechanism which needs to re-arm itself automatically.
     */
    void HandleAutomatic(void);
    
    /*! \brief
     *  \param unused
     *
     */
    static void CallHandleAutomatic(void * unused);

    /*! \brief A method to run the robot lift motor action
     *  \todo Write this method
     */
    //void Lift(void);

    /*! \brief Get the angle from the gyro
     *
     *  This method is used to get the angle information from the
     *  gyro onboard the robot
     *
     *  \todo Positioning system - gyro, accelerometer
     */
    //float GetAngle(void);
};

#endif  // ROBOT980_H
