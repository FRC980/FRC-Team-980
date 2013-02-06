#include "MyRobot.h"
#include <math.h>

#define RUN_ONCE_VAR(joystick,button,var)              \
    static bool var = false;                           \
    if (! joystick->GetRawButton(button))              \
    {                                                  \
         var = false;                                  \
    }                                                  \
    else if (joystick->GetRawButton(button) &&         \
             !var && (var=true))

#define RUN_ONCE(joystick,button)                      \
    RUN_ONCE_VAR(joystick,button,joystick##_##button##_pressed)

typedef enum 
{
    AUTON_DRIVE,
    AUTON_LOWER_ARM
} auton_state_t;


void message(char *fmt, ...)
{
    char message[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(message, 256, fmt, args);
    va_end(args);

    setErrorData(message, strlen(message), 100);
}

double limit(double val, double min = -1, double max = 1)
{
    if (val > max)
        return max;
    if (val < min)
        return min;

    return val;
}

MyRobot::MyRobot(void) 
    : m_pscLeft1(new CANJaguar(CAN_LEFT_CIM1)), 
      m_pscLeft2(new CANJaguar(CAN_LEFT_CIM2)),
      m_pscRight1(new CANJaguar(CAN_RIGHT_CIM1)),
      m_pscRight2(new CANJaguar(CAN_RIGHT_CIM2)),
      m_pJoystick1(new Joystick(1)),
      m_pSteeringwheel(new Joystick(2)),
      m_pCompressor(new Compressor(3, 1)),
      m_pTestValveA(new Solenoid(1)),
      m_pTestValveB(new Solenoid(2))
{
    m_pscLeft1->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
    m_pscLeft1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscLeft1->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
    m_pscLeft1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscLeft2->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
    m_pscLeft2->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscLeft2->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
    m_pscLeft2->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscRight1->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
    m_pscRight1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscRight1->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
    m_pscRight1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscRight2->ConfigEncoderCodesPerRev(US_DIGITAL_ENC_COUNTS);
    m_pscRight2->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscRight2->ConfigMaxOutputVoltage(MAX_JAGUAR_OUTPUT_VOLTAGE);
    m_pscRight2->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
}

MyRobot::~MyRobot(void) {
    delete m_pscLeft1;
    delete m_pscLeft2;
    delete m_pscRight1;
    delete m_pscRight2;
    delete m_pJoystick1;
    delete m_pSteeringwheel;
    delete m_pCompressor;
    delete m_pTestValveA;
    delete m_pTestValveA;
}

void MyRobot::Autonomous(void) {

}

void MyRobot::OperatorControl(void) {
    GetWatchdog().SetEnabled(true);

    m_pCompressor->Start();

    while (IsOperatorControl() && IsEnabled()) {
        // feeding the watchdog
        GetWatchdog().Feed();

        //******************************** DRIVE *************************************
        float gain, throttle;

        gain = m_pSteeringwheel->GetX();
        throttle = m_pJoystick1->GetY();

	    float eGain = (gain > 0) ? gain : gain * -1;
    	float eThrottle = (throttle > 0) ? throttle : throttle * -1;
        //2.71828183
	    // eGain = pow(2.7, (2.4*eGain-3));
	    // eThrottle = pow(2.7, (3*eThrottle-3));
    	eGain = pow(2.71828183, (2.4*eGain-3));
    	eThrottle = pow(2.71828183, (3*eThrottle-3));

        if(gain > -0.03 && gain < 0.03)
            eGain = 0;
        gain = (gain > 0) ? eGain : (eGain)* -1;
        
        if(throttle > -0.05 && throttle < 0.05)
            eThrottle = 0;
        throttle = (throttle > 0) ? (eThrottle)* -1 : (eThrottle);
        //set default fLeft and fRight to throttle
	    float fLeft = throttle;
        float fRight = fLeft;

        //if statements for distributing power to left and right depending on gain value 
    	if (gain>0.05 || gain<-0.05)
        {	        
            fLeft = throttle+(gain);
	        fRight = throttle-(gain);
        }

        //Drive(fLeft, fRight);

        //******************************** TEST *************************************
        RUN_ONCE(m_pJoystick1, 2) 
        {
            message("close");
            m_pTestValveA->Set(true);
            m_pTestValveB->Set(false);
        }

        RUN_ONCE(m_pJoystick1, 3) 
        {
            message("open");
            m_pTestValveB->Set(true);
            m_pTestValveA->Set(false);
        }

        Wait(0.05);
    }
}

void MyRobot::Drive(float right, float left) {
    m_pscLeft1->Set(limit(left));
    m_pscLeft2->Set(limit(left));
    m_pscRight1->Set(limit(-right));
    m_pscRight2->Set(limit(-right));
}

START_ROBOT_CLASS(MyRobot)
