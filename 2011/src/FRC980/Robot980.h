#ifndef ROBOT980_H
#define ROBOT980_H

//--- Include the appropriate libraries
#include "numbers.h" // This includes numerical constants used by the robot

#include "CANJaguar.h" // This includes the necessary methods to use the CAN Jaguar
//#include <Vision/PCVideoServer.h>

// REVIEW: Style -- include full header, or just use a forward class
// declaration?  Most of these get pulled in from "CANJaguar.h".

// class CANJaguar;
// class DigitalInput;
// class Notifier;
// class PCVideoServer;
// class Timer;
// class Victor;

//==========================================================================
/*
 * FRC Team 980 - Wiring Diagram
 *
 * Jaguars
 * =======
 *
 * Left Drive CIM 1   --> Jag11
 * Left Drive CIM 2   --> Jag12
 * Right Drive CIM 1  --> Jag13
 * Right Drive CIM 2  --> Jag14
 * Minibot Deployment --> Jag15
 * Arm-Claw Motor     --> Jag20 (black)
 *
 *
 * Victors
 * =======
 *
 * Arm-Shoulder Motor 1 --> Victor1
 * Arm-Shoulder Motor 2 --> Victor2
 *
 *
 * Digital Side Car
 * ================
 *
 * PWM Channels
 * ------------
 * Arm-Shoulder Motors 1/2  --> DSC D/O 1
 *
 * Sensors
 * --------------
 * Line Sensor - Left    --> DSC D/I 1
 * Line Sensor - Center  --> DSC D/I 2
 * Line Sensor - Right   --> DSC D/I 3
 *
 *
 * cRIO
 * ====
 *
 * Ethernet Port 1   -> robot radio (wireless bridge)
 *
 * Gyro              -> Analog input 1
 * Autonomous switch -> Analog input 7
 *
 * Options:
 * 1) Camera -> cRIO Ethernet Port 2
 * 2) Camera -> robot radio (wireless bridge)
 *
 *
 * Classmate PC
 * ============
 * Login: Driver; NO PASSWORD
 * Login: Developer; NO PASSWORD
 *
 * Classmate Ethernet Port    --> unused
 * 4-Port USB Expansion Black --> Left Classmate USB
 * 4-Port USB Expansion Red   --> Right Classmate USB
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

/*! \def TOP_SPEED
 *  Theoretical speed of CIM is 5500 RPM
 *  free-running top speed (in ft/sec) = (speed of cim in rpm) / 60sec/min
 *  (ratio of toughbox) * (sprocket ratio) * pi * (wheel diameter in feet)
 */
const double TOP_SPEED = ((double)5500 / (double)60 / (GEARBOX_RATIO) * (GEAR_RATIO) * M_PI * (double)0.5);     /* ~ 17 ft/sec */

//Wheel diameter in inches
#define WHEEL_DIAMETER (6.0)

//==============================================================================
// CAN Jaguar Outputs
#define CAN_LEFT_DRIVE1             11  /*!< \def CAN_LEFT_DRIVE1 The CAN Jaguar device number for the Left Drive Motor */
#define CAN_LEFT_DRIVE2             12  /*!< \def CAN_LEFT_DRIVE2 The CAN Jaguar device number for the Left Drive Motor */
#define CAN_RIGHT_DRIVE1            13  /*!< \def CAN_RIGHT_DRIVE1 The CAN Jaguar device number for the Right Drive Motor */
#define CAN_RIGHT_DRIVE2            14  /*!< \def CAN_RIGHT_DRIVE2 The CAN Jaguar device number for the Right Drive Motor */
#define CAN_MINIDEPLOY              15  /*!< \def CAN_MINIDEPLOY The CAN Jaguar device number for the minibot deployment */
#define CAN_ARM_CLAW                20  /*!< \def CAN_ARM_CLAW The CAN Jaguar device number for the claw on the arm */

// Jaguar Outputs
#define MAX_JAGUAR_OUTPUT_VOLTAGE   12.0        /*!< \def MAX_JAGUAR_OUTPUT_VOLTAGE The maximum output voltage of the CAN Jaguar */
// Encoder setup information
#define US_DIGITAL_ENC_COUNTS       360 /*!< \def US_DIGITAL_ENC_COUNTS The number of encoder counts for the US Digital Encoders */

//==============================================================================
// Digital Side Car Outputs

// Digital Side Car Slot
#define DSC_SLOT                    4   /*!< \def DSC_SLOT The slot number in the cRio for the Digital Side Car */

// PWM outputs
#define CHAN_PWM_SHOULDER           1   /*!< \def CHAN_PWM_SHOULDER The Digital Side Car PWM Channel for both of the Shoulder Motors */


//==============================================================================
// Digital Inputs

// Line tracking sensors
#define CHAN_LINE_LEFT              1   /*!< \def CHAN_LINE_SENSOR_LEFT Input for left line sensor */
#define CHAN_LINE_CENTER            2   /*!< \def CHAN_LINE_SENSOR_CENTER Input for center line sensor */
#define CHAN_LINE_RIGHT             3   /*!< \def CHAN_LINE_SENSOR_RIGHT Input for right line sensor */

// Digital Outputs
#define CHAN_LIGHT_TRIANGLE         12  
#define CHAN_LIGHT_CIRCLE           13
#define CHAN_LIGHT_SQUARE           14

//==============================================================================
// Analog Inputs
#define SLOT_GYRO                   1   /*!< SLOT_GYRO The slot number in the cRio for the Gyro analog input*/
#define CHAN_GYRO                   1   /*!< CHAN_GYRO The Analog Channel for the Gyro. FPGA requires gyro to be on chan 1 */

#define SLOT_AUTO_MODE              1   /*!< SLOT_AUTO_MODE The slot number in the cRio for the Auto Mode analog input */
#define CHAN_AUTO_MODE              7   /*!< CHAN_AUTO_MODE The Analog Channel for the Auto Mode. */

#define ANALOG_SLOT      1
#define CHAN_ARM_POTENTIOMETER      4

#define POT_PID_P                   0.2/30.0
#define POT_PID_I                   0.0
#define POT_PID_D                   0.0
#define POT_TOLERANCE               4.0 /* % */


#define POT_LOWER_LIMIT             40

#define POT_GROUND                  40

#define POT_SIDE_LOW                130
#define POT_SIDE_MIDDLE             285
#define POT_SIDE_HIGH               430

#define POT_CENTER_LOW              175
#define POT_CENTER_MIDDLE           300
#define POT_CENTER_HIGH             475

#define POT_CARRY                   500
#define POT_VERTICAL                10000

#define POT_UPPER_LIMIT             700

//==============================================================================
// Define Additional Values

// all switches should be wired the same -- either normal open = high, or
// normal open = low.  In our case, normal open = low(?)
#define SW_OPEN     true
#define SW_CLOSED   false

//--- NULL Value
#ifndef NULL
#define NULL                        (0) /*!< \def NULL The NULL value is created here because it is not created elsewhere */
#endif  // NULL

//==========================================================================
typedef enum {
    LED_OFF = 0,
    LED_RED = 1,
    LED_TRIANGLE = 1,
    LED_WHITE = 2,
    LED_CIRCLE = 2,
    LED_BLUE = 3,
    LED_SQUARE = 3
} LED_t;

//==========================================================================
//! Code specific to the Team 980 Robot
/*!\class Robot980
 *
 * The purpose of the Robot980 class is to contain code specific to the
 * team's robot.
 *
 */
class Robot980 : public SensorBase
{
  private:
    //--- Instance Variables -----------------------------------------------
    //--- Jaguars
    // left and right drive motors
    CANJaguar *m_pscLeft1;   /*!< The 1st Left Drive motor speed controller */
    CANJaguar *m_pscLeft2;   /*!< The 2nd Left Drive motor speed controller */
    CANJaguar *m_pscRight1;  /*!< The 1st Right Drive motor speed controller */
    CANJaguar *m_pscRight2;  /*!< The 2nd Right Drive motor speed controller */
    CANJaguar *m_pscMiniDeploy; /*!< The minibot deploy motor speed controller */
    CANJaguar *m_pscClaw;    /*!< The arm's claw motor speed controller */

    //--- Victors
public:
    Victor *m_pscShoulder; /*!< The shoulder motor speed controller */
private:
    //--- Digital Outputs (Lights)
    DigitalOutput* m_pdoLightTriangle;
    DigitalOutput* m_pdoLightCircle;
    DigitalOutput* m_pdoLightSquare;

    //--- Sensors
    Gyro* m_pGyro;                 /*!< The Gyro Sensor */
    DigitalInput *m_pdiLineLeft;   /*!< Line Sensor Left */
    DigitalInput *m_pdiLineCenter; /*!< Line Sensor Center */
    DigitalInput *m_pdiLineRight;  /*!< Line Sensor Right */
    AnalogChannel* m_pacAutonSwitch;
    AnalogChannel* m_pacArmPosition;
    
    //--- PIDs
    PIDController* m_pidArm;

    // more sensors TBD

    //--- Timers
    Timer *m_pTimerDrive; /*!< The Timer used for debugging (calc & print speeds) */
    Timer *m_pTimerClaw;

    //--- Notifiers
    Notifier *m_pNotifierClaw;

  private:
    // PCVideoServer* m_pVideoServer;

    //--- Constructors ----------------------------------------------------
    /*! \brief The Robot 980 Constructor
     *
     *  The constructor/destructor are private to enforce this being a
     *  singleton object, accessible only via Robot980::GetInstance();
     */
    Robot980();

    //--- Destructors -----------------------------------------------------
    /*! \brief The Robot 980 Destructor
     *
     *  The constructor/destructor are private to enforce this being a
     *  singleton object, accessible only via Robot980::GetInstance();
     */
    virtual ~Robot980();

  public:
    //--- Constructors ----------------------------------------------------
    /*! \brief Robot980 is a singleton class -- get the instance of it
     */
    static Robot980 *GetInstance();

    //--- Methods ---------------------------------------------------------
    /*!\brief The accessor method for the autonomous mode
     *
     *  There will be a multi-position switch wired as a potentiometer,
     *  connected to a single analog input.  GetAutonMode reads that
     *  analog value and converts it to a single "mode" integer.
     */
    int  GetAutonMode(void);

    /*! \brief Set the Jaguars to either brake mode or coast mode
     */
    void SetBrakes(bool brakeOnStop);

    /*! \brief A drive method to move the robot
     *  \param left The speed of the left drive motors
     *  \param right The speed of the right drive motors
     *
     *  This method is used to drive the robot.
     *
     *  For reference: 1 = forward, -1 = backwards
     */
    void Drive(float left, float right);
    
    /*! \brief A method to move the arm up and down
     *  \param position The target arm position
     */
    void SetPosition(int position);

    void SetArmSpeed(float speed);

    //! \brief Get current arm position
    int GetPosition();


    //! \brief Get data from line tracker
    //  \return (left, middle, right) booleans stored as char
    char GetLineTracker(bool invert = false);

    //! \briefSet the LED color
    // 0=none, 1=triangle, 2=circle, 3=square \todo use enum
    void LightLED(LED_t led);

    //! \brief Get right encoder value
    float GetRightEncoder();

    //! \brief Get right encoder value
    float GetLeftEncoder();

    /*! \brief Print a debug message displaying the current status
     */
    void PrintState(void);

    /*! \brief Get the angle from the gyro
     *
     *  This method is used to get the angle information from the gyro
     *  onboard the robot
     *
     *  \todo Positioning system - gyro, accelerometer
     */
    float GetAngle(void);

    /*! \brief Send command to jaguar to open claw
     */
    void OpenClaw(float speed = 1.0);

    /*! \brief Send command to jaguar to close claw
     */
    void CloseClaw(float speed = 0.8);

    /*! \brief Stop the claw if needed (called by notifier)
     */
    static void CheckClaw(void* pvRobot);

    /*! \brief Run the deployment motor
     */
    void Deploy(float speed);
};

#endif  // ROBOT980_H
