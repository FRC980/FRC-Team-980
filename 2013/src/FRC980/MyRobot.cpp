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
    : m_pscLeft(new Victor(CHAN_PWM_LEFT_DRIVE)), 
      m_pscRight(new Victor(CHAN_PWM_RIGHT_DRIVE)),
      m_pDriveShiftA(new Solenoid(SOLENOID_SLOT1, CHAN_SOL_DRIVE_SHIFT_A)),
      m_pDriveShiftB(new Solenoid(SOLENOID_SLOT1, CHAN_SOL_DRIVE_SHIFT_B)),
	  m_pClawTopA(new Solenoid(SOLENOID_SLOT1, CHAN_SOL_CLAW_TOP_A)),
	  m_pClawTopB(new Solenoid(SOLENOID_SLOT1, CHAN_SOL_CLAW_TOP_B)),
	  m_pClawBottomA(new Solenoid(SOLENOID_SLOT1, CHAN_SOL_CLAW_BOTTOM_A)),
	  m_pClawBottomB(new Solenoid(SOLENOID_SLOT1, CHAN_SOL_CLAW_BOTTOM_B)),
      m_pscClimbTop(new Jaguar(ID_JAG_CLIMB_TOP)),
	  m_pscClimbBottom(new Jaguar(ID_JAG_CLIMB_BOTTOM)),
      m_pJoystick1(new Joystick(1)),
      m_pSteeringwheel(new Joystick(2)),
      m_pCompressor(new Compressor(CHAN_COMP_AUTO_SHUTOFF, CHAN_RLY_COMPRESSOR))
{
}

MyRobot::~MyRobot(void) {
    delete m_pscLeft;
    delete m_pscRight;
    delete m_pDriveShiftA;
    delete m_pDriveShiftB;
	delete m_pClawTopA;
	delete m_pClawTopB;
	delete m_pClawBottomA;
	delete m_pClawBottomB;
	delete m_pscClimbTop;
	delete m_pscClimbBottom;
    delete m_pJoystick1;
    delete m_pSteeringwheel;
    delete m_pCompressor;
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

        if(gain > -0.03 && gain < 0.03) {
            eGain = 0;
	    }
        gain = (gain > 0) ? eGain : (eGain)* -1;
        
        if(throttle > -0.05 && throttle < 0.05) {
            eThrottle = 0;
	    }
        throttle = (throttle > 0) ? (eThrottle)* -1 : (eThrottle);
        
	    //set default fLeft and fRight to throttle
	    float fLeft = throttle;
        float fRight = fLeft;

        //if statements for distributing power to left and right depending on gain value 
    	if (gain>0.05 || gain<-0.05) {	        
            fLeft = throttle+(gain);
	    fRight = throttle-(gain);
        }

        //Drive(fLeft, fRight);
        Wait(0.05);
    }
}

void MyRobot::Drive(float right, float left) {
    m_pscLeft->Set(limit(left));
    m_pscRight->Set(limit(-right));
}

void MyRobot::ShiftDrive(bool high) {
    if(high) {
        m_pDriveShiftA->Set(true);
        m_pDriveShiftB->Set(false);
    } else {
        m_pDriveShiftA->Set(false);
        m_pDriveShiftB->Set(true);
    }
}


START_ROBOT_CLASS(MyRobot)
