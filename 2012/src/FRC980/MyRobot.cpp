#include "iostream"
#include "fstream"
#include "WPILib.h"
#include "MyRobot.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
#include "Cyclops.h"
#include <math.h>

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
    : m_pscShooterMaster(new CANJaguar(12, CANJaguar::kSpeed))
    , m_pscShooterSlave1(new CANJaguar(14, CANJaguar::kVoltage))
    , m_pscLeft1(new CANJaguar(11))
    , m_pscRight1(new CANJaguar(13))
    , m_pscBallPickup(new Victor(1))
    , m_pscBallFeeder(new Victor(2))
    , m_pscBridge(new Victor(3))
    , joystick1(new MyJoystick(1))
    , joystick2(new MyJoystick(3))
    , steeringwheel(new MyJoystick(2)) 
    , ds(DriverStation::GetInstance())
    , m_bridge_timer(new Timer())
{
    m_pscLeft1->ConfigEncoderCodesPerRev(360);
    m_pscLeft1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscLeft1->ConfigMaxOutputVoltage(12.0);
    m_pscLeft1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscRight1->ConfigEncoderCodesPerRev(360);
    m_pscRight1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscRight1->ConfigMaxOutputVoltage(12.0);
    m_pscRight1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscShooterMaster->ConfigEncoderCodesPerRev(250);
    m_pscShooterMaster->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
    m_pscShooterMaster->ConfigMaxOutputVoltage(12);
    
    float p, i, d;
    p = 0.027;
    i = 0.0;
    d = 0.0;

    m_pscShooterMaster->SetPID(p,i,d);
    m_pscShooterMaster->EnableControl();

    m_bridge_timer->Reset();
    m_bridge_timer->Start();
}

MyRobot::~MyRobot(void)
{
    delete m_pscRight1;
    delete m_pscLeft1;
    delete m_pscShooterMaster;
    delete m_pscShooterSlave1;
    delete m_pscBallPickup;
    delete m_pscBallFeeder;
    delete m_pscBridge;
    delete joystick1;
    delete joystick2;
    delete steeringwheel;
    delete ds;
    delete m_bridge_timer;
}

void MyRobot::Autonomous(void)
{
    GetWatchdog().SetEnabled(false);
    
    Timer timer;
    timer.Reset();
    timer.Start();

    int auton_state = 4;
    float target_distance = GetLeftEncoder()-5;
    float distance = GetLeftEncoder();

    float speed = 0.0;
    float setspeed = 2*2300;

    switch(auton_state)
    {
        case 1:
            while (timer.Get() < 0.2)
            {
                Drive(.50, .50);
            }
            while (timer.Get() < 0.4)
            {
                Drive(-.30, -.30);
            }
            Drive(0.0,0.0);
            break;

        case 2:
            setspeed = 2*2300;
            timer.Reset();
            timer.Start();
            while (timer.Get() < 4)
            {
                Drive(0.35, 0.35);
                distance = GetLeftEncoder();
                SetShooterSpeed(speed);
                if(speed <= setspeed)
                    speed+=setspeed/20.0;

            }
            Drive(0.0, 0.0);
            while(timer.Get() < 8)
            {
                SetShooterSpeed(speed);
                m_pscBallPickup->Set(-1.0);
                m_pscBallFeeder->Set(-1.0);
            }
            break;
        
        case 3:
            while (distance > target_distance)
            {
                Drive(0.35, 0.35);
                distance = GetLeftEncoder();
            }
            Drive(0.0,0.0);
            break;
        case 4:
            setspeed = 2*1900;
            timer.Reset();
            timer.Start();
            while (timer.Get() < 1)
            {

            }
            while (timer.Get() < 5)
            {
                Drive(0.35, 0.35);
                distance = GetLeftEncoder();
                SetShooterSpeed(speed);
                if(speed <= setspeed)
                    speed+=setspeed/20.0;

            }
            Drive(0.0, 0.0);
            while(timer.Get() < 9)
            {
                SetShooterSpeed(speed);
                m_pscBallPickup->Set(-1.0);
                m_pscBallFeeder->Set(-1.0);
            }
            break;
    }
}

void MyRobot::OperatorControl(void)
{
    //Cyclops *cyclops;
    TargetAlignment alignment;
    unsigned int distance;

    // Commented out to get rid of unused compile warning.
    const float targetspeed = 2300.0;
    float speed = 0.0;

    GetWatchdog().SetEnabled(true);

    //cyclops = new Cyclops();
    //cyclops->Start();

    SetBrakes(false);
    DriveControlMode(false);

    ofstream myfile;
    myfile.open("example.txt");

    Timer timer;
    timer.Reset();
    timer.Start();

    while (IsOperatorControl() && IsEnabled())
    {   
        float setspeed = 2*targetspeed;
        GetWatchdog().Feed();
        
        //double acceleration_x, acceleration_y, acceleration_z;
        //acceleration_x = m_pAccelerometer->GetAcceleration(ADXL345_I2C::kAxis_X);
        //acceleration_y = m_pAccelerometer->GetAcceleration(ADXL345_I2C::kAxis_Y);
        //acceleration_z = m_pAccelerometer->GetAcceleration(ADXL345_I2C::kAxis_Z);

       // message("x: %f, y: %f, z: %f", acceleration_x, acceleration_y, acceleration_z);

	 //Drive
    	//initialize gain and throttle varaibles
        float gain, throttle;

        gain = steeringwheel->GetX();
        throttle = joystick1->GetY();

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

        Drive(fLeft, fRight);

        /*
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
        */

        float x2 = joystick2->GetX();
        float y2 = joystick2->GetY();

        SetShooterSpeed(speed);
        if(speed <= setspeed)
            speed+=setspeed/20.0;
        else
            speed = setspeed + (2000*x2);

	    //to get x and y from joystick2 via button press on joystick1
	
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

        if(joystick2->GetRawButton(9))
        {
            setspeed = 2300; 
        }
        if(joystick2->GetRawButton(11))
        {
            setspeed = 2500;
        }

        static bool bridgeup = true;
        
        if(joystick2->GetRawButton(2) && bridgeup)
        {
            bridgeup = false;
            RunBridge(true); 
        }
        else if(!joystick2->GetRawButton(2) && !bridgeup)
        {
            bridgeup = true;
            RunBridge(false);
        }
        
        CheckStopBridge(); 


        if(joystick2->GetRawButton(4))
	    {
	        m_pscBallPickup->Set(-1.0);
	    }
        else
        {
            m_pscBallPickup->Set(0.0);
        }

        if(joystick2->GetRawButton(8))
        {
           m_pscBallFeeder->Set(-1.0); 
        }
        else
        {
            if (joystick2->GetRawButton(3))
            {
                m_pscBallFeeder->Set(.77);
            }
            else if (joystick2->GetRawButton(5))
            {
                m_pscBallFeeder->Set(-1.0);
            }
            else
            {
                m_pscBallFeeder->Set(0.0);
            }
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

    //cyclops->Stop();
    //delete cyclops;
}

void MyRobot::CheckStopBridge(void)
{
    if(m_bridge_timer->Get() > 0.5)
    {
        m_pscBridge->Set(0.0);
    }
}

void MyRobot::RunBridge(bool up)
{
   m_bridge_timer->Reset();
   if(up)
   {
       m_pscBridge->Set(-0.3);
   }
   else
       m_pscBridge->Set(0.3);
}


float MyRobot::GetRPM(void)
{
    return m_pscShooterMaster->GetSpeed();
}

void MyRobot::Drive(float left, float right)
{
    m_pscLeft1->Set(limit(left));

    m_pscRight1->Set(limit(-right));
}

void MyRobot::DriveControl(float position_right, float position_left)
{
    m_pscLeft1->Set(position_left);
    
    m_pscRight1->Set(position_right);
}

void MyRobot::SetShooterSpeed(float speed)
{
    m_pscShooterMaster->Set(speed);
    float voltage = m_pscShooterMaster->GetOutputVoltage();
    m_pscShooterSlave1->Set(voltage);
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

}

void MyRobot::PerformBalanceTrick(MyJoystick *joy)
{
    DriveControlMode(true);
    
    float initial_position_right = GetRightEncoder();
    float initial_position_left = GetLeftEncoder();
    float target_position_right = initial_position_right+360;
    float target_position_left = initial_position_left-360;

    while (IsOperatorControl() && IsEnabled())
    {
        GetWatchdog().Feed();

        DriveControl(target_position_right, target_position_left);
        
        RUN_ONCE(joystick1, 8)
        {
            message("left encoder: %f", GetLeftEncoder());
        }

        RUN_ONCE(joystick1, 9)
        {
            message("right encoder: %f", GetRightEncoder());
        }
        
        RUN_ONCE(joystick1, 2)
        {
           
        }
    }
    
    DriveControlMode(false);
}

void MyRobot::DriveControlMode(bool control)
{
    if (control)
    {
        m_pscLeft1->ChangeControlMode(CANJaguar::kPosition);
        m_pscRight1->ChangeControlMode(CANJaguar::kPosition);
        float p, i, d;
        p = -0.2;
        i = 0.0;
        d = 0;

        m_pscLeft1->SetPID(p,i,d);
        m_pscRight1->SetPID(p,i,d);
        m_pscLeft1->EnableControl();
        m_pscRight1->EnableControl();
        message("drive control enabled");
    }
    else
    {
        m_pscLeft1->DisableControl();
        m_pscRight1->DisableControl();
        m_pscLeft1->ChangeControlMode(CANJaguar::kPercentVbus);
        m_pscRight1->ChangeControlMode(CANJaguar::kPercentVbus);
        message("drive control disabled");
    }
}

START_ROBOT_CLASS(MyRobot)
