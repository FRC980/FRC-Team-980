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
#define CAN_LEFT_DRIVE1               3   /*!< \def CAN_LEFT_DRIVE1 The CAN Jaguar device number for the Left Drive Motor */
//#define CAN_LEFT_DRIVE2             12  /*!< \def CAN_LEFT_DRIVE2 The CAN Jaguar device number for the Left Drive Motor */
//#define CAN_RIGHT_DRIVE1            13  /*!< \def CAN_RIGHT_DRIVE1 The CAN Jaguar device number for the Right Drive Motor */
//#define CAN_RIGHT_DRIVE2            14  /*!< \def CAN_RIGHT_DRIVE2 The CAN Jaguar device number for the Right Drive Motor */

// Jaguar Outputs
#define MAX_JAGUAR_OUTPUT_VOLTAGE   12.0        /*!< \def MAX_JAGUAR_OUTPUT_VOLTAGE The maximum output voltage of the CAN Jaguar */
// Encoder setup information
#define US_DIGITAL_ENC_COUNTS       360 /*!< \def US_DIGITAL_ENC_COUNTS The number of encoder counts for the US Digital Encoders */

//==============================================================================
// Digital Side Car Outputs

// Digital Side Car Slot
#define DSC_SLOT                    2   /*!< \def DSC_SLOT The slot number in the cRio for the Digital Side Car */


//==============================================================================
// Digital Inputs


//==============================================================================
// Analog Inputs

#define SLOT_AUTO_MODE              1   /*!< SLOT_AUTO_MODE The slot number in the cRio for the Auto Mode analog input */
#define CHAN_AUTO_MODE              7   /*!< CHAN_AUTO_MODE The Analog Channel for the Auto Mode. */

#define ANALOG_SLOT      1

//==============================================================================
// Define Additional Values

//--- NULL Value
#ifndef NULL
#define NULL                        (0) /*!< \def NULL The NULL value is created here because it is not created elsewhere */
#endif  // NULL

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
    //CANJaguar *m_pscLeft1;   /*!< The 1st Left Drive motor speed controller */
    //CANJaguar *m_pscLeft2;   /*!< The 2nd Left Drive motor speed controller */
    //CANJaguar *m_pscRight1;  /*!< The 1st Right Drive motor speed controller */
    //CANJaguar *m_pscRight2;  /*!< The 2nd Right Drive motor speed controller */

    //--- Victors
    //Victor *m_victor; /*!< The shoulder motor speed controller */

    //--- Sensors

    // more sensors TBD

    //--- Timers
	Timer *m_pTimerDrive;
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
    void Drive(float num);

    //! \brief Get right encoder value
    float GetEncoder();

};

#endif  // ROBOT980_H
