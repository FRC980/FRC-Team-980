#include "iostream"
#include "fstream"
#include "WPILib.h"
#include "MyRobot.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Cyclops.h"
#include <math.h>
#include "jsbuttons.h"

/*
 * TODO: The sensitivity and zero values vary by accelerometer model. 
 *       There are constants defined for various models.
 * Set the CORRECT values, Craig 3/13.
 */
#define ACCEL_SENSITIVITY 0.0
#define ACCEL_ZERO	  0.0

#define ANALOG_CHANNEL_ACCEL 7

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
    : m_pscLeft1(new CANJaguar(11))
    , m_pscRight1(new CANJaguar(13))
    , joystick1(new MyJoystick(1))
    , steeringwheel(new MyJoystick(2)) 
{
    m_pscLeft1->ConfigEncoderCodesPerRev(360);
    m_pscLeft1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscLeft1->ConfigMaxOutputVoltage(12.0);
    m_pscLeft1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscRight1->ConfigEncoderCodesPerRev(360);
    m_pscRight1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscRight1->ConfigMaxOutputVoltage(12.0);
    m_pscRight1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
}

MyRobot::~MyRobot(void)
{
    delete m_pscRight1;
    delete m_pscLeft1;
    delete joystick1;
    delete steeringwheel;
    delete ds;
}

void MyRobot::Autonomous(void)
{
}

void MyRobot::OperatorControl(void)
{   
    GetWatchdog().SetEnabled(true);
    int DriveMode = PercentVBus;
    SetBrakes(false);
    DriveControlMode(CANJaguar::kPercentVbus);

    while (IsOperatorControl() && IsEnabled())
    {  
    	
        //float setspeed = 2*targetspeed+(joystick2->GetY()*700);
        GetWatchdog().Feed();

	//Drive ---------------------------------------------------------------------
    	//initialize gain and throttle varaibles
        float gain, throttle;

        gain = steeringwheel->GetX();
        throttle = joystick1->GetY();

       	float eGain = (gain > 0) ? gain : gain * -1;
        float eThrottle = (throttle > 0) ? throttle : throttle * -1;

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

	if(DriveMode == Speed)
	{
	    fLeft = fLeft;
	    fRight = fRight;
	    message("right: %f, left %f", GetSpeedRight(), GetSpeedLeft());
	    DriveControlSpeed(fLeft, fRight);
	}
	else if(DriveMode == PercentVBus)
	{
            Drive(fLeft, fRight);
	}
	
	RUN_ONCE(joystick1, 1)
	{
	    if(DriveMode == Speed)
	    {
		DriveMode = PercentVBus;
		DriveControlMode(CANJaguar::kPercentVbus);
	    }
	    else if(DriveMode == PercentVBus)
	    {
		DriveMode = Speed;
		DriveControlMode(CANJaguar::kSpeed);
	    }
	}

	RUN_ONCE(joystick1, 4)
	{
	    message("Left Encoder: %f", GetLeftEncoder());
	}

	RUN_ONCE(joystick1, 5)
	{
	    message("Right Encoder: %f", GetRightEncoder());
	}

        Wait(0.05);
    }
}

void MyRobot::Drive(float left, float right)
{
    m_pscLeft1->Set(limit(left));

    m_pscRight1->Set(limit(-right));
}

void MyRobot::DriveControlPosition(float position_right, float position_left)
{
    m_pscLeft1->Set(position_left);
    
    m_pscRight1->Set(position_right);
}

void MyRobot::DriveControlSpeed(float speed_right, float speed_left)
{
    m_pscLeft1->Set(speed_left);
    
    m_pscRight1->Set(-speed_right);
}

float MyRobot::GetRightEncoder(void)
{
    return m_pscRight1->GetPosition();
}   

float MyRobot::GetLeftEncoder(void)
{
    return m_pscLeft1->GetPosition();
}

float MyRobot::GetSpeedRight(void)
{
    return m_pscRight1->GetSpeed();
}

float MyRobot::GetSpeedLeft(void)
{
    return m_pscLeft1->GetSpeed();
}

void MyRobot::SetBrakes(bool brakeOnStop)
{
    CANJaguar::NeutralMode mode = brakeOnStop
        ? CANJaguar::kNeutralMode_Brake : CANJaguar::kNeutralMode_Coast;

    m_pscLeft1->ConfigNeutralMode(mode);
    m_pscRight1->ConfigNeutralMode(mode);

}

void MyRobot::DriveControlMode(CANJaguar::ControlMode control)
{
    float p, i, d;
    m_pscLeft1->DisableControl();
    m_pscRight1->DisableControl();

    switch (control) {
    case CANJaguar::kPosition:
    	m_pscLeft1->ConfigEncoderCodesPerRev(1);
        m_pscLeft1->ChangeControlMode(CANJaguar::kPosition);
    	m_pscRight1->ConfigEncoderCodesPerRev(1);
        m_pscRight1->ChangeControlMode(CANJaguar::kPosition);
        m_pscRight1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        m_pscLeft1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
        p = -0.2;
        i = -0.00217;
        d = 0;

        m_pscLeft1->SetPID(p,i,d);
        m_pscRight1->SetPID(p,i,d);
        m_pscLeft1->EnableControl();
        m_pscRight1->EnableControl();
        message("drive control enabled");
	    break;
    
    case CANJaguar::kSpeed:
    	m_pscLeft1->ConfigEncoderCodesPerRev(360);
    	m_pscLeft1->ChangeControlMode(CANJaguar::kSpeed);
        m_pscLeft1->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
    	m_pscRight1->ConfigEncoderCodesPerRev(360);
        m_pscRight1->ChangeControlMode(CANJaguar::kSpeed);
	m_pscRight1->SetSpeedReference(CANJaguar::kSpeedRef_QuadEncoder);
        p = 0.350;
        i = 0.004;
        d = 0.000;

        m_pscLeft1->SetPID(p,i,d);
        m_pscRight1->SetPID(p,i,d);
        m_pscLeft1->EnableControl();
        m_pscRight1->EnableControl();
        message("drive control enabled");
        break;

    case CANJaguar::kPercentVbus:
    	m_pscLeft1->ConfigEncoderCodesPerRev(360);
        m_pscLeft1->ChangeControlMode(CANJaguar::kPercentVbus);
    	m_pscRight1->ConfigEncoderCodesPerRev(360);
        m_pscRight1->ChangeControlMode(CANJaguar::kPercentVbus);
        message("drive control disabled");
        break;

    case CANJaguar::kVoltage:
    case CANJaguar::kCurrent:
    default:
    	break;
    }
}

int MyRobot::GetAutonMode(void)
{
    AnalogModule *pAM = AnalogModule::GetInstance(1);
    int i = pAM->GetValue(7);

    if (i > 900)
        return 6;
    if (i > 700)
        return 5;
    if (i > 500)
        return 4;
    if (i > 300)
        return 3;
    if (i > 100)
        return 2;

    return 1;
}

START_ROBOT_CLASS(MyRobot)
