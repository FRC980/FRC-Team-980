#include "iostream"
#include "fstream"
#include "WPILib.h"
#include "MyRobot.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Cyclops.h"
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
    : m_pscShooterMaster(new CANJaguar(15, CANJaguar::kSpeed))
    , m_pscShooterSlave1(new CANJaguar(16, CANJaguar::kVoltage))
    , m_pscShooterSlave2(new CANJaguar(17, CANJaguar::kVoltage))
    , m_pscShooterSlave3(new CANJaguar(18, CANJaguar::kVoltage))
    , m_pscLeft1(new CANJaguar(11))
    , m_pscLeft2(new CANJaguar(12))
    , m_pscRight1(new CANJaguar(13))
    , m_pscRight2(new CANJaguar(14))
    , m_pscBallPickup(new Victor(1))
    , m_pscBallFeeder(new Victor(2))
    , m_pscTurret(new Victor(3))
    , joystick1(new Joystick(1))
    , joystick2(new Joystick(3))
    , steeringwheel(new Joystick(2)) 
    , ds(DriverStation::GetInstance())
    , m_pAccelerometer(new ADXL345_I2C(7))
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
    p = 0.029;
    i = 0.0;
    d = 0.0;

    m_pscShooterMaster->SetPID(p,i,d);
    m_pscShooterMaster->EnableControl();

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
    //float posRight = GetRightEncoder();
    float targetLeft = posLeft+1;
    //float targetRight = posRight+1;
    
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
    Cyclops *cyclops;
    float p, i, d;
    TargetAlignment alignment;
    unsigned int distance;

    // Commented out to get rid of unused compile warning.
    // const float targetspeed = 3000.0;
    // float setspeed = 2*targetspeed;
    // float speed = 0.0;

    p = 0.029;
    i = 0.0;
    d = 0.0;

    GetWatchdog().SetEnabled(true);

    cyclops = new Cyclops();
    cyclops->Start();

    SetBrakes(false);
    
    ofstream myfile;
    myfile.open("example.txt");

    Timer timer;
    timer.Reset();
    timer.Start();

    while (IsOperatorControl() && IsEnabled())
    {   
        GetWatchdog().Feed();

	//Drive
    	//initialize gain and throttle varaibles
        float gain, throttle;

        gain = steeringwheel->GetX();
        throttle = joystick1->GetY();

	float eGain = (gain > 0) ? gain : gain * -1;
	float eThrottle = (throttle > 0) ? throttle : throttle * -1;

	eGain = pow(2.71828183, (2.4*eGain-3));
	eThrottle = pow(2.71828183, (3*eThrottle-3));

        gain = (gain > 0) ? eGain : (eGain)* -1;
        throttle = (throttle > 0) ? (eThrottle)* -1 : (eThrottle);
	if (throttle < 0.08 && throttle > -0.08)
	{
	    throttle = 0.0;
	}
	//set default fLeft and fRight to throttle
	float fLeft = throttle;
        float fRight = fLeft;

    //if statements for distributing power to left and right depending on gain value 
	if (gain>0.05 || gain<-0.05)
	{
	    fLeft = throttle+(gain);
	    fRight = throttle-(gain);
	}

        Drive(fLeft, fRight);

	//target finder
        RUN_ONCE(joystick1, 1)
        {
            message("finding targets");
	        distance = cyclops->GetDistanceToTarget();
	        alignment = cyclops->IsTargetAligned();

	        message("distance from target: %f", distance);
	        switch (alignment) {
		    case TARGET_LEFT:
		        message("you are off to the left");
		        break;
		    case TARGET_RIGHT:
		        message("you are off to the right");
		        break;
		    case TARGET_ALIGNED:
		        message("you are on target");
		        break;
		    case TARGET_UNKNOWN:
		        message("Not all targets visible");
		        break;
            }
            message("done");
        }

       	/* 
        if(joystick1->GetRawButton(3))
        {
            SetShooterSpeed(speed);
            if(speed <= setspeed)
                speed+=setspeed/20.0;
        }
        else
        {
            SetShooterSpeed(0);
            speed = 0.0;
        }
	*/
	//to get x and y from joystick2 via button press on joystick1
	
        if (joystick1->GetRawButton(2))
        {
            if (myfile.is_open())
                myfile << "speed: " << GetRPM() << ", target: 2000, p: " << p << ", i: " << i << ", d: " << d << ", timer: " << timer.Get() << endl;
            message("speed: %f", GetRPM());
        }

        RUN_ONCE(joystick1, 4)
        {
            SetBrakes(true);
            message("brakes set: on");
        }

        RUN_ONCE(joystick1, 5)
        {
            SetBrakes(false);
            message("brakes set: off");
        }

        RUN_ONCE(joystick1, 11)
        {
            m_pscShooterMaster->DisableControl();
            i += 0.0001;
            message("i: %f", i);
            m_pscShooterMaster->SetPID(p,i,d);
            m_pscShooterMaster->EnableControl();
        }

        RUN_ONCE(joystick1, 10)
        {
            m_pscShooterMaster->DisableControl();
            i -= 0.0001;
            message("i: %f", i);
            m_pscShooterMaster->SetPID(p,i,d);
            m_pscShooterMaster->EnableControl();
        }

        RUN_ONCE(joystick1,6)
        {
            m_pscShooterMaster->DisableControl();
            p += 0.001;
            message("p: %f", p);
            m_pscShooterMaster->SetPID(p,i,d);
            m_pscShooterMaster->EnableControl();
        }

        RUN_ONCE(joystick1,7)
        {
            m_pscShooterMaster->DisableControl();
            p -= 0.001;
            message("p: %f", p);
            m_pscShooterMaster->SetPID(p,i,d);
            m_pscShooterMaster->EnableControl();
        }

        RUN_ONCE(joystick1, 3)
        {
            DoBalance();
        }

        /*
        if(joystick2->GetRawButton(5))
	    {
	        m_pscBallPickup->Set(1.0);
	    }
	    else if(joystick2->GetRawButton(4))
	    {
	        m_pscBallPickup->Set(-1.0);
	    }
        else
        {
            m_pscBallPickup->Set(0.0);
        }

        if (joystick2->GetRawButton(3))
        {
            m_pscBallFeeder->Set(1.0);
        }
        else if (joystick2->GetRawButton(2))
        {
            m_pscBallFeeder->Set(-1.0);
        }
        else
        {
            m_pscBallFeeder->Set(0.0);
        }

        if (joystick2->GetRawButton(6))
        {
            m_pscTurret->Set(0.15);
        }
        else if (joystick2->GetRawButton(7))
        {
            m_pscTurret->Set(-0.15);
        }
        else
        {
            m_pscTurret->Set(0.0);
        }

        RUN_ONCE(joystick2, 8)
        {
            message("left encoder: %f", GetLeftEncoder());
        }

        RUN_ONCE(joystick2, 9)
        {
            message("right encoder: %f", GetRightEncoder());
        } 
*/       
///////////////////this if for joystick2 button test/////////////////////

        if (joystick2->GetRawButton(1))
        {
            message("joystick2: Button 1");
        }
	
        if (joystick2->GetRawButton(2))
        {
            message("joystick2: Button 2");
        }

        if (joystick2->GetRawButton(3))
        {
            message("joystick2: Button 3");
        }

        if (joystick2->GetRawButton(4))
        {
            message("joystick2: Button 4");
        }
        
        RUN_ONCE(joystick2, 5)
        {
            message("joystick2: Button 5");
        }
	
        RUN_ONCE(joystick2, 6)
        {
            message("joystick2: Button 6");
        }
        
        RUN_ONCE(joystick2, 7)
        {
            message("joystick2: Button 7");
        }
        
        RUN_ONCE(joystick2, 8)
        {
            message("joystick2: Button 8");
        }
        
        RUN_ONCE(joystick2, 9)
        {
            message("joystick2: Button 9");
        }
        
        RUN_ONCE(joystick2, 10)
        {
            message("joystick2: Button 10");
        }
        
        RUN_ONCE(joystick2, 11)
        {
            message("joystick2: Button 11");
        }
        
        Wait(0.05);
    }

    cyclops->Stop();
    delete cyclops;
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

void MyRobot::DriveControl(float position_right, float position_left)
{
    m_pscLeft1->Set(position_left);
    float voltage_left = m_pscLeft1->GetOutputVoltage();
    m_pscLeft2->Set(voltage_left);
    
    m_pscRight1->Set(position_right);
    float voltage_right = m_pscRight1->GetOutputVoltage();
    m_pscRight2->Set(voltage_right);
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

void MyRobot::SetBrakes(bool brakeOnStop)
{
    CANJaguar::NeutralMode mode = brakeOnStop
        ? CANJaguar::kNeutralMode_Brake : CANJaguar::kNeutralMode_Coast;

    m_pscLeft1->ConfigNeutralMode(mode);
    m_pscRight1->ConfigNeutralMode(mode);
    m_pscRight2->ConfigNeutralMode(mode);
    m_pscLeft2->ConfigNeutralMode(mode);

}

void MyRobot::DoBalance()
{
    bool done = false;
    DriveControlMode(true);
    
    float initial_position_right = GetRightEncoder();
    float initial_position_left = GetLeftEncoder();
    float target_position_right = initial_position_right;
    float target_position_left = initial_position_left;

    while (!done)
    {
        GetWatchdog().Feed();

        DriveControl(target_position_right, target_position_left);

        float x = joystick1->GetX();
        float y = joystick1->GetY();
        if (x > 0.02 || x < -0.02 || y > 0.02 || y < -0.02)
        {
            done = true;
        }

        RUN_ONCE(joystick1, 2)
        {
            double acceleration = m_pAccelerometer->GetAcceleration(ADXL345_I2C::kAxis_Z);
            if (acceleration > 0)
            {
                target_position_right+=0.08;
                target_position_left+=0.08;
            }
            else if (acceleration < 0)
            {
                target_position_right-=0.08;
                target_position_left-=0.08;
            }
        }
    }
    
    DriveControlMode(false);
}

void MyRobot::DriveControlMode(bool control)
{
    if (control)
    {
        m_pscLeft1->ChangeControlMode(CANJaguar::kPosition);
        m_pscLeft2->ChangeControlMode(CANJaguar::kVoltage);
        m_pscRight1->ChangeControlMode(CANJaguar::kPosition);
        m_pscRight2->ChangeControlMode(CANJaguar::kVoltage);
        float p, i, d;
        p = i = d = 0.0;

        m_pscLeft1->SetPID(p,i,d);
        m_pscRight1->SetPID(p,i,d);
        m_pscLeft1->EnableControl();
        m_pscLeft1->EnableControl();
    }
    else
    {
        m_pscLeft1->DisableControl();
        m_pscRight1->DisableControl();
        m_pscLeft1->ChangeControlMode(CANJaguar::kPercentVbus);
        m_pscLeft2->ChangeControlMode(CANJaguar::kPercentVbus);
        m_pscRight1->ChangeControlMode(CANJaguar::kPercentVbus);
        m_pscRight2->ChangeControlMode(CANJaguar::kPercentVbus);
    }
}

START_ROBOT_CLASS(MyRobot)
