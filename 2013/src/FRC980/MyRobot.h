#include "WPILib.h"

#ifndef MyRobot_h_
#define MyRobot_h_

//==============================================================================
// CAN Jaguar Outputs
#define CAN_LEFT_CIM1               11  /*!< \def CAN_LEFT_CIM1 The CAN Jaguar device number for the Left CIM Motor */
#define CAN_LEFT_CIM2               12  /*!< \def CAN_LEFT_CIM2 The CAN Jaguar device number for the Left CIM Motor */
#define CAN_RIGHT_CIM1              13  /*!< \def CAN_RIGHT_CIM1 The CAN Jaguar device number for the Right CIM Motor */
#define CAN_RIGHT_CIM2              14  /*!< \def CAN_RIGHT_CIM2 The CAN Jaguar device number for the Right CIM Motor */

// Jaguar Outputs
#define MAX_JAGUAR_OUTPUT_VOLTAGE   12.0        /*!< \def MAX_JAGUAR_OUTPUT_VOLTAGE The maximum output voltage of the CAN Jaguar */
// Encoder setup information
#define US_DIGITAL_ENC_COUNTS       4028 /*!< \def US_DIGITAL_ENC_COUNTS The number of encoder counts for the US Digital Encoders */

//==============================================================================

class MyRobot : public SimpleRobot
{
private:
    CANJaguar* m_pscLeft1;    
    CANJaguar* m_pscLeft2;    

    CANJaguar* m_pscRight1;    
    CANJaguar* m_pscRight2;

    Joystick *m_pJoystick1;
    Joystick *m_pSteeringwheel;

    Compressor *m_pCompressor;
    int pressureSwitchValue;

    Solenoid *m_pTestValveA;
    Solenoid *m_pTestValveB;

public:
    MyRobot(void);
    ~MyRobot(void);
    void Autonomous(void);
    void OperatorControl(void);
    void Drive(float, float);
};

#endif
