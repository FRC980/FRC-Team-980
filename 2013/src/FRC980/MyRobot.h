#include "WPILib.h"

#ifndef MyRobot_h_
#define MyRobot_h_

//==============================================================================
/*
 * FRC Team 980 - Wiring Diagram
 *
 * Jaguars
 * =======
 *
 * Lower Climing Wheel Deploy  --> Jag11
 * Upper Climing Wheel Deploy  --> Jag12
 *
 *
 * Victors
 * =======
 *
 * Left Drive CIM 1     --> Victor1 
 * Left Drive CIM 2     --> Victor2 
 * Right Drive CIM 1    --> Victor3 
 * Right Drive CIM 2    --> Victor4 
 * Shooter Crank Moter  --> Victor5
 *
 *
 * Digital Side Car
 * ================
 *
 * PWM Channels
 * ------------
 * Left Drive    --> DSC D/O 1
 * Right Drive   --> DSC D/O 2
 * Shooter Crank --> DSC D/O 3
 *
 * Limit Switches
 * --------------
 * // Example: Arming Switch (detects kicker in armed position) --> DSC D/I 1
 *
 *
 * Classmate PC
 * ============
 * Login: Driver; NO PASSWORD
 * Login: Developer; NO PASSWORD
 *
 *
 * External Accessories
 * ====================
 *
 * Joystick 1  --> USB Expansion Port
 * Joystick 2  --> USB Expansion Port
 *
 */

//==============================================================================

//==============================================================================
// CAN Jaguar Outputs
#define ID_JAG_CLIMB_TOP			11
#define ID_JAG_CLIMB_BOTTOM			12

// Jaguar Outputs
#define MAX_JAGUAR_OUTPUT_VOLTAGE   12.0        /*!< \def MAX_JAGUAR_OUTPUT_VOLTAGE The maximum output voltage of the CAN Jaguar */
// Encoder setup information
#define US_DIGITAL_ENC_COUNTS       4028 /*!< \def US_DIGITAL_ENC_COUNTS The number of encoder counts for the US Digital Encoders */

//==============================================================================
// Digital Side Car Outputs

// Digital Side Car Slot
#define DSC_SLOT                           4

// PWM outputs
#define CHAN_PWM_LEFT_DRIVE                1  /*!< \def  CHAN_PWM_LEFT_DRIVE The PWM Victor device number for the Left CIM Motor */
#define CHAN_PWM_RIGHT_DRIVE               2  /*!< \def CHAN_PWM_RIGHT_DRIVE The PWM Victor device number for the Left CIM Motor */

// Relay outputs
#define CHAN_RLY_COMPRESSOR                1  /*!< \def CHAN_RLY_COMPRESSOR The PWM Spike device number for the compressor */

//==============================================================================
// Digital Inputs

// Compressor auto shut off pressure switch
#define CHAN_COMP_AUTO_SHUTOFF             1

//==============================================================================
// Solenoid Module Outputs

// Solenoid Module Slots
#define SOLENOID_SLOT1                     1
#define SOLENOID_SLOT2                     2

// Solenoid outputs
#define CHAN_SOL_DRIVE_SHIFT_A             1
#define CHAN_SOL_DRIVE_SHIFT_B             2
#define CHAN_SOL_CLAW_TOP_A                3
#define CHAN_SOL_CLAW_TOP_B                4
#define CHAN_SOL_CLAW_BOTTOM_A             5
#define CHAN_SOL_CLAW_BOTTOM_B             6

// Number of Solenoids
#define NUM_SOLENOIDS                      7

//==============================================================================

class MyRobot : public SimpleRobot
{
private:
    Victor* m_pscLeft;    

    Victor* m_pscRight;   

    Solenoid m_pValves[NUM_SOLENOIDS];

    Solenoid *m_pDriveShiftA;
    Solenoid *m_pDriveShiftB;
	Solenoid *m_pClawTopA;
	Solenoid *m_pClawTopB;
	Solenoid *m_pClawBottomA;
	Solenoid *m_pClawBottomB;

	Jaguar* m_pscClimbTop;
	Jaguar* m_pscClimbBottom; 

    Joystick *m_pJoystick1;
    Joystick *m_pSteeringwheel;

    Compressor *m_pCompressor;

public:
    MyRobot(void);
    ~MyRobot(void);
    void Autonomous(void);
    void OperatorControl(void);
    void Drive(float, float);
    void ShiftDrive(bool);
};

#endif
