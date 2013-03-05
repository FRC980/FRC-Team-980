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
      m_pscWinch(new Victor(CHAN_PWM_WINCH)),
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
    delete m_pscWinch;

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
    Timer timer;
    timer.Reset();
    timer.Start();

    while(timer.Get() < 4.0f) {
    	if(timer.Get() < 1.0f) {
	    RunWinch(1.0f);
	} else if(timer.Get() >= 1.0f && timer.Get() <= 2.0f) {
	    RunWinch(-1.0f);
	} else {
	    RunWinch(0.0f);
	}

	Drive(1.0f, -1.0f);
    }

    Drive(0.0, 0.0);

    
    
    while(timer.Get() > 5.0f && timer.Get() < 8.0f) {
	
    }
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

void MyRobot::RunWinch(float speed) {
    m_pscWinch->Set(limit(speed));
}

void MyRobot::Fire(void) {
    OpenValve(SOL_CATAPOLT_RELEASE);
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

bool MyRobot::CheckStopBottomWheel() {
    float t = m_pTimerBottomWheel->Get();

    if(t < 0.25) {
        return false;
    }

    if(m_pscClimbBottom->GetOutputCurrent() > 3.0) {
        m_pscClimbBottom->Set(0.0);
        return true;
    }

    return false;
}

void MyRobot::EngageTopWheel() {
    m_pscClimbTop->Set(1.0);
    m_pTimerTopWheel->Reset();
}

void MyRobot::DisengageTopWheel() {
    m_pscClimbTop->Set(-1.0);
    m_pTimerTopWheel->Reset();
}

bool MyRobot::CheckStopTopWheel() {
    float t = m_pTimerTopWheel->Get();

    if(t < 0.25) {
        return false;
    }

    if(m_pscClimbTop->GetOutputCurrent() > 3.0) {
        m_pscClimbTop->Set(0.0);
        return true;
    }
    
    return false;
}

void MyRobot::ClimbAuto(void) {
#define CLIMB_AUTO_INIT         1
#define CLOSE_BOTTOM_CLAW       2
#define DEPLOY_BOTTOM_WHEEL     3
#define DRIVE_TO_NEXT_LEVEL     4
#define CLOSE_TOP_CLAW          11
#define DEPLOY_TOP_WHEEL        5
#define DETRACT_BOTTOM_WHEEL    6
#define OPEN_BOTTOM_CLAW        7
#define DRIVE_OVER_BAR          8
#define DETRACT_TOP_WHEEL       9
#define OPEN_TOP_CLAW           10

    int state = CLIMB_AUTO_INIT;
    int level = 1;
    bool driveToNextLevel = true;
    Timer timer;
    timer.Reset();
    bool RunOnceBool = true;
    bool atTop = false;

    while(!atTop) {
        GetWatchdog().Feed();
        switch(state) {
            case CLIMB_AUTO_INIT:
                if(RunOnceBool) {
                    OpenValve(SOL_START_CLIMB); 
                    CloseValve(SOL_DRIVE_SHIFT);
                    timer.Reset();
                    RunOnceBool = false;
                }
                if(timer.Get() >= 5.0) {
                    state = CLOSE_BOTTOM_CLAW;
                    RunOnceBool = true;
                }
                break;
            case CLOSE_BOTTOM_CLAW:
                OpenValve(SOL_CLAW_BOTTOM);
                state = DEPLOY_BOTTOM_WHEEL;
                break;
            case DEPLOY_BOTTOM_WHEEL:
                if(RunOnceBool) {
                    EngageBottomWheel();
                    RunOnceBool = false;
                }
                if(CheckStopBottomWheel()) {
                    if(driveToNextLevel) {
                        state = DRIVE_TO_NEXT_LEVEL;
                        driveToNextLevel = false;
                    } else {
                        state = DETRACT_TOP_WHEEL;
                        driveToNextLevel = true;
                    }
                    RunOnceBool = true;
                }
                break;
            case DRIVE_TO_NEXT_LEVEL:
                Drive(1.0, -1.0);
                if(false) { // name of limit switch not known
                    Drive(0.0, 0.0);
                    state = CLOSE_TOP_CLAW;
                }
                break;
            case CLOSE_TOP_CLAW:
                OpenValve(SOL_CLAW_TOP);
                state = DEPLOY_TOP_WHEEL;
                break;
            case DEPLOY_TOP_WHEEL:
                if (RunOnceBool) {
                    EngageTopWheel();
                    RunOnceBool = false;
                }
                if (CheckStopTopWheel()) {
                    state = DETRACT_BOTTOM_WHEEL;
                    RunOnceBool = true;
                }
                break;
            case DETRACT_BOTTOM_WHEEL:
                if (RunOnceBool) {
                    DisengageBottomWheel();
                    RunOnceBool = false;
                }
                if (CheckStopBottomWheel()) { // Make sure that is correct
                    state = OPEN_BOTTOM_CLAW;
                    RunOnceBool = true;
                }
                break;
            case OPEN_BOTTOM_CLAW:
                CloseValve(SOL_CLAW_BOTTOM);
                state = DRIVE_OVER_BAR;
                break;
            case DRIVE_OVER_BAR:
                Drive(1.0, -1.0);
                if (false) { // name of limit switch not known
                    Drive(0.0, 0.0);
                    state = CLOSE_BOTTOM_CLAW;
                    level++;
                    if(level >= 3) {
                        atTop = true;
                    }
                }
                break;
            case DETRACT_TOP_WHEEL:
                if (RunOnceBool) {
                    DisengageTopWheel();
                    RunOnceBool = false;
                }
                if (CheckStopTopWheel()) {
                    state = OPEN_TOP_CLAW;
                    RunOnceBool = true;
                }
                break;
            case OPEN_TOP_CLAW:
                CloseValve(SOL_CLAW_TOP);
                state = DRIVE_TO_NEXT_LEVEL;
                break;
        }

        RUN_ONCE(m_pJoystick1, 1) {
            return;
        }
    }
}

START_ROBOT_CLASS(MyRobot)
