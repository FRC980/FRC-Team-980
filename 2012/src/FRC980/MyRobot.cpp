#include "WPILib.h"
#include "MyRobot.h"


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
    : m_pscShooterMaster(new CANJaguar(15, CANJaguar::kSpeed))
    , m_pscShooterSlave1(new CANJaguar(16, CANJaguar::kVoltage))
    , m_pscShooterSlave2(new CANJaguar(17, CANJaguar::kVoltage))
    , m_pscShooterSlave3(new CANJaguar(18, CANJaguar::kVoltage))
    , m_pscLeft1(new CANJaguar(14))
    , m_pscLeft2(new CANJaguar(13))
    , m_pscRight1(new CANJaguar(12))
    , m_pscRight2(new CANJaguar(11))
    , m_pscBallPickup(new Victor(1))
    , m_pscBallFeeder(new Victor(2))
    , m_pscTurret(new Victor(3))
    , joystick1(new Joystick(1))
    , steeringwheel(new Joystick(2)) 
    , ds(DriverStation::GetInstance())
{
    m_pscLeft1->ConfigEncoderCodesPerRev(360);
    m_pscLeft1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscLeft1->ConfigMaxOutputVoltage(12.0);
    m_pscLeft1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscLeft2->ConfigMaxOutputVoltage(12.0);
    m_pscLeft2->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscRight1->ConfigEncoderCodesPerRev(360);
    m_pscRight1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscRight1->ConfigMaxOutputVoltage(12.0);
    m_pscRight1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);
 
    m_pscRight2->ConfigMaxOutputVoltage(12.0);
    m_pscRight2->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);   

    m_pscShooterMaster->ConfigEncoderCodesPerRev(250);
    m_pscShooterMaster->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
    m_pscShooterMaster->ConfigMaxOutputVoltage(12);
    
    float p, i, d;
    p = 0.001;
    i = 0.9;
    d = 0.0;

    m_pscShooterMaster->SetPID(p,i,d);
    m_pscShooterMaster->EnableControl();

    //increased the SetExpiration(float) from 0.1 to 0.25 
    //to attemt to fix watchdog not fed error
    GetWatchdog();
}

MyRobot::~MyRobot(void)
{
    delete m_pscRight1;
    delete m_pscRight2;
    delete m_pscLeft1;
    delete m_pscLeft2;
    delete m_pscShooterMaster;
    delete m_pscShooterSlave1;
    delete m_pscShooterSlave2;
    delete m_pscShooterSlave3;
    delete m_pscBallPickup;
    delete m_pscBallFeeder;
    delete m_pscTurret;
    delete joystick1;
    delete steeringwheel;
    delete ds;
}

void MyRobot::Autonomous(void)
{
    GetWatchdog().SetEnabled(false);

    auton_state_t auton_state = AUTON_DRIVE;

    float posLeft = GetLeftEncoder();
    float posRight = GetRightEncoder();
    float targetLeft = posLeft+1;
    float targetRight = posRight+1;
    
    while (auton_state == AUTON_DRIVE)
    {
        posLeft = GetLeftEncoder();
        if (posLeft < targetLeft)
        {
            Drive(0.35,0.35);
        }
    }
}

void MyRobot::OperatorControl(void)
{
    GetWatchdog().SetEnabled(true);

    while (IsOperatorControl())
    {   
        GetWatchdog().Feed();

    	//initialize gain and throttle varaibles
        float gain, throttle;
        gain = steeringwheel->GetX();
        throttle = joystick1->GetY();

        gain = (gain > 0) ? gain * gain : gain * gain * -1;
        throttle = (throttle > 0) ? throttle * throttle * -1 : throttle * throttle;
	//set default fLeft and fRight to throttle
        float fLeft = throttle;
        float fRight = throttle;
	
        //if statements for distributing power to 
	//left and right depending on gain value
	if (gain>0.05)
	{
	    fLeft = throttle+gain*2.0;
	    fRight = throttle-gain*2.0;
	}
	else if (gain<-0.05)
	{
	    fLeft = throttle+gain*2.0;
	    fRight = throttle-gain*2.0;
	}

        Drive(fLeft, fRight);

        if (joystick1->GetRawButton(5))
	{
	    m_pscBallPickup->Set(1.0);
	}
        else if (joystick1->GetRawButton(4))
	{
	    m_pscBallPickup->Set(-1.0);
	}
        else 
	{
            m_pscBallPickup->Set(0.0);
        }

        if (joystick1->GetRawButton(3))
        {
            m_pscBallFeeder->Set(1.0);
        }
        else if (joystick1->GetRawButton(2))
        {
            m_pscBallFeeder->Set(-1.0);
        }
        else
        {
            m_pscBallFeeder->Set(0.0);
        }

        if (joystick1->GetRawButton(6))
        {
            m_pscTurret->Set(0.15);
        }
        else if (joystick1->GetRawButton(7))
        {
            m_pscTurret->Set(-0.15);
        }
        else
        {
            m_pscTurret->Set(0.0);
        }

        RUN_ONCE(joystick1, 8)
        {
            message("left encoder: %f", GetLeftEncoder());
        }

        RUN_ONCE(joystick1, 9)
        {
            message("right encoder: %f", GetRightEncoder());
        } 
        
        Wait(0.05);
    }
}

float MyRobot::GetRPM(void)
{
    return m_pscShooterMaster->GetSpeed();
}

void MyRobot::Drive(float left, float right)
{
    m_pscLeft1->Set(limit(left));
    m_pscLeft2->Set(limit(left));

    m_pscRight1->Set(limit(-right));
    m_pscRight2->Set(limit(-right));
}

void MyRobot::SetShooterSpeed(float speed)
{
    m_pscShooterMaster->Set(speed);
    float voltage = m_pscShooterMaster->GetOutputVoltage();
    m_pscShooterSlave1->Set(voltage);
    m_pscShooterSlave2->Set(voltage);
    m_pscShooterSlave3->Set(voltage);
}

float MyRobot::GetRightEncoder(void)
{
    return m_pscRight1->GetPosition();
}   

float MyRobot::GetLeftEncoder(void)
{
    return m_pscLeft1->GetPosition();
}

START_ROBOT_CLASS(MyRobot)
