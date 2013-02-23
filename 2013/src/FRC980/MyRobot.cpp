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
      m_pscClimbTop(new CANJaguar(ID_JAG_CLIMB_TOP)),
	  m_pscClimbBottom(new CANJaguar(ID_JAG_CLIMB_BOTTOM)),
      m_pJoystick1(new Joystick(1)),
      m_pSteeringwheel(new Joystick(2)),
      m_pCompressor(new Compressor(CHAN_COMP_AUTO_SHUTOFF, CHAN_RLY_COMPRESSOR)),
      m_pTimerTopWheel(new Timer),
      m_pTimerBottomWheel(new Timer)
{
    m_pValves[SOL_DRIVE_SHIFT] = new Solenoid(SOLENOID_SLOT1, CHAN_SOL_DRIVE_SHIFT_A);
    m_pValves[SOL_DRIVE_SHIFT+1] = new Solenoid(SOLENOID_SLOT1, CHAN_SOL_DRIVE_SHIFT_B);
    m_pValves[SOL_CLAW_TOP] = new Solenoid(SOLENOID_SLOT1, CHAN_SOL_CLAW_TOP_A);
    m_pValves[SOL_CLAW_TOP+1] = new Solenoid(SOLENOID_SLOT1, CHAN_SOL_CLAW_TOP_B);
    m_pValves[SOL_CLAW_BOTTOM] = new Solenoid(SOLENOID_SLOT1, CHAN_SOL_CLAW_BOTTOM_A);
    m_pValves[SOL_CLAW_BOTTOM+1] = new Solenoid(SOLENOID_SLOT1, CHAN_SOL_CLAW_BOTTOM_B);

    m_pTimerTopWheel->Reset();
    m_pTimerTopWheel->Start();

    m_pTimerBottomWheel->Reset();
    m_pTimerBottomWheel->Start();
}

MyRobot::~MyRobot(void) {
    delete m_pscLeft;
    delete m_pscRight;

    for(int i = 0; i < NUM_SOLENOIDS; i++) {
        delete m_pValves[i];
    }

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

    CloseValve(SOL_CLAW_TOP);
    CloseValve(SOL_CLAW_BOTTOM);

    bool topClawClosed = false;
    bool bottomClawClosed = false;
    bool topWheelEngaged = false;
    bool bottomWheelEngaged = false;

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

        Drive(fLeft, fRight);

        // Test controls for valves
        // valve for shifting
        RUN_ONCE(m_pJoystick1, 2) {
            OpenValve(SOL_DRIVE_SHIFT);
        }
        RUN_ONCE(m_pJoystick1, 3) {
            CloseValve(SOL_DRIVE_SHIFT);
        }
        
        // valve for top claw
        RUN_ONCE(m_pJoystick1, 6) {
            OpenValve(SOL_CLAW_TOP);
            topClawClosed = true;
        }
        RUN_ONCE(m_pJoystick1, 7) {
                CloseValve(SOL_CLAW_TOP);
                topClawClosed = false;
        }
        
        // valve for bottom claw
        RUN_ONCE(m_pJoystick1, 11) {
            OpenValve(SOL_CLAW_BOTTOM);
            bottomClawClosed = true;
        }
        RUN_ONCE(m_pJoystick1, 10) {
                CloseValve(SOL_CLAW_BOTTOM);
                bottomClawClosed = false;
        }

        // Test controls for climb wheels
        // bottom wheel
        static bool bottom_wheel_engaged = false;

        RUN_ONCE(m_pJoystick1, 8) {
            if(!bottom_wheel_engaged) {
                EngageBottomWheel();
                bottom_wheel_engaged = true;
            }
        }
        RUN_ONCE(m_pJoystick1, 9) {
            if(bottom_wheel_engaged) {
                DisengageBottomWheel();
                bottom_wheel_engaged = false;
            }
        }

        CheckStopBottomWheel();
        
        // top wheel
        static bool top_wheel_engaged = false;

        RUN_ONCE(m_pJoystick1, 4) {
            if(!top_wheel_engaged) {
                EngageTopWheel();
                top_wheel_engaged = true;
            }
        }
        RUN_ONCE(m_pJoystick1, 5) {
            if(top_wheel_engaged) {
                DisengageTopWheel();
                top_wheel_engaged = false;
            }
        }

        CheckStopTopWheel();


        /*
        DigitalInput *magSwitch = new DigitalInput(2);

        if(magSwitch->Get()) {
            message("Piston In");
        } else {
            message("Piston Out");
        }

        delete magSwitch;
        */

        Wait(0.05);
    }
}

void MyRobot::Drive(float right, float left) {
    m_pscLeft->Set(limit(left));
    m_pscRight->Set(limit(-right));
}

void MyRobot::OpenValve(int valve) {
    m_pValves[valve]->Set(false);
    m_pValves[valve+1]->Set(true);
}

void MyRobot::CloseValve(int valve) { 
    m_pValves[valve]->Set(true);
    m_pValves[valve+1]->Set(false);

}

void MyRobot::EngageBottomWheel() {
    m_pscClimbBottom->Set(1.0);
    m_pTimerBottomWheel->Reset();
}

void MyRobot::DisengageBottomWheel() {
    m_pscClimbBottom->Set(-1.0);
    m_pTimerBottomWheel->Reset();
}

void MyRobot::CheckStopBottomWheel() {
    float t = m_pTimerBottomWheel->Get();

    if(t < 0.25) {
        return;
    }

    if(m_pscClimbBottom->GetOutputCurrent() > 3.0) {
        m_pscClimbBottom->Set(0.0);
    }
}

void MyRobot::EngageTopWheel() {
    m_pscClimbTop->Set(1.0);
    m_pTimerTopWheel->Reset();
}

void MyRobot::DisengageTopWheel() {
    m_pscClimbTop->Set(-1.0);
    m_pTimerTopWheel->Reset();
}

void MyRobot::CheckStopTopWheel() {
    float t = m_pTimerTopWheel->Get();

    if(t < 0.25) {
        return;
    }

    if(m_pscClimbTop->GetOutputCurrent() > 3.0) {
        m_pscClimbTop->Set(0.0);
    }
}

START_ROBOT_CLASS(MyRobot)
