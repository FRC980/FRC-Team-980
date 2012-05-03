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

/*
 * Values in feet
 */
#define TIRE_DIAMETER 0.5208
#define DISTANCE_BETWEEN_WHEELS 1.7916

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
    : m_pscShooterMaster(new CANJaguar(15))
    , m_pscShooterSlave1(new CANJaguar(17))
    , m_pscLeft1(new CANJaguar(14))
    , m_pscRight1(new CANJaguar(12))
    , m_pscBridge(new CANJaguar(18))
    , m_pscBallPickup(new Victor(1))
    , m_pscBallFeeder(new Victor(2))
    , joystick1(new MyJoystick(1))
    , joystick2(new MyJoystick(3))
    , steeringwheel(new MyJoystick(2)) 
    , ds(DriverStation::GetInstance())
    , m_peShooter(new Encoder(1,1,1,2, false, Encoder::k4X))
    //, m_pAccelerometer(new ADXL345_I2C(1)) 
{
    m_pscLeft1->ConfigEncoderCodesPerRev(250);
    m_pscLeft1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscLeft1->ConfigMaxOutputVoltage(12.0);
    m_pscLeft1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscRight1->ConfigEncoderCodesPerRev(250);
    m_pscRight1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    m_pscRight1->ConfigMaxOutputVoltage(12.0);
    m_pscRight1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

    m_pscBridge->ConfigNeutralMode(CANJaguar::kNeutralMode_Brake);
    
    m_peShooter->SetDistancePerPulse(1.0/360.0);
    m_peShooter->Reset();
    m_peShooter->Start();


    float p, i, d;
    p = 0.0315;
    i = 0.0;
    d = 0.0;
}

MyRobot::~MyRobot(void)
{
    delete m_pscRight1;
    delete m_pscLeft1;
    delete m_pscShooterMaster;
    delete m_pscShooterSlave1;
    delete m_pscBridge;
    delete m_pscBallPickup;
    delete m_pscBallFeeder;
    delete joystick1;
    delete joystick2;
    delete steeringwheel;
    delete ds;
    delete m_peShooter;
    //delete m_pAccelerometer;
}

void MyRobot::Autonomous(void)
{
    GetWatchdog().SetEnabled(false);
    DriveControlMode(CANJaguar::kPercentVbus);

    m_peShooter->Reset();
    m_peShooter->Start();

    Timer timer;
    timer.Reset();
    timer.Start();

    int auton_state = GetAutonMode();
    float distance = GetLeftEncoder();

    float speed = 0.0;
    float setspeed = 2*2300;

    float target_distance;

    switch(auton_state)
    {
        case 1: //High shots from fender
            setspeed = 2*FENDER_HIGH;
            target_distance = GetLeftEncoder()-6.5;

            timer.Reset();
            timer.Start();

            while(GetLeftEncoder() > target_distance && timer.Get() < 5)
            {
                Drive(0.35, 0.365);
                message("encoder: %f", GetLeftEncoder());
                message("target: %f", target_distance);
                
                if(speed < setspeed)
                {
                    speed+=100;
                }

                SetShooterSpeed(speed);
            }

            Drive(0.0, 0.0);

            while(timer.Get() < 12)
            {
                if(speed < setspeed)
                {
                    speed+=100;
                }
                SetShooterSpeed(speed);
                m_pscBallPickup->Set(-1.0);
                m_pscBallFeeder->Set(-1.0);
            }
            break;
        
        case 2: //Medium shots from fender
            setspeed = 2*FENDER_MEDIUM;
            target_distance = GetLeftEncoder()-6.5;

            timer.Reset();
            timer.Start();

            while(GetLeftEncoder() > target_distance && timer.Get() < 5)
            {
                Drive(0.35, 0.365);
                message("encoder: %f", GetLeftEncoder());
                message("target: %f", target_distance);
                if(speed < setspeed)
                {
                    speed+=100;
                }
                SetShooterSpeed(speed);
            }

            Drive(0.0, 0.0);

            while(timer.Get() < 9)
            {
                SetShooterSpeed(speed);
                m_pscBallPickup->Set(-1.0);
                m_pscBallFeeder->Set(-1.0);
            }
            break;
        case 3: //Back up/Bridge
            target_distance = GetLeftEncoder()+4;
            
            while(GetLeftEncoder() < target_distance)
            {
                Drive(-0.48, -0.48);
                message("encoder: %f", GetLeftEncoder());
                message("target: %f", target_distance);
                RunBridge(true); 
            }

            timer.Reset();
            timer.Start();

            Drive(0.0, 0.0);

            while(timer.Get() < 3)
            {
            }

            while(timer.Get() < 4)
            {
                RunBridge(false);
            }
            
            break;
        case 4:
            timer.Reset();
            timer.Start();

            while(timer.Get() < 4)
            {
            }

            m_pscBallPickup->Set(1.0);
            break;
    }
}

void MyRobot::OperatorControl(void)
{
    //Cyclops *cyclops;
    float distance;
    float angle;

    //high-goal fender speed = 2300
    //medium-goal fender speed = 1800

    // Commented out to get rid of unused compile warning.
    float targetspeed = FENDER_HIGH;
    float speed = 0.0;
    bool windup = true;

    GetWatchdog().SetEnabled(false);

    //cyclops = new Cyclops();
    //cyclops->Start();

    SetBrakes(false);
    DriveControlMode(CANJaguar::kPercentVbus);

    ofstream myfile;
    myfile.open("example.txt");

    Timer timer;
    timer.Reset();
    timer.Start();

    while (IsOperatorControl() && IsEnabled())
    {   
        float setspeed = targetspeed+(joystick2->GetX()*700);
        // GetWatchdog().Feed();

        //myfile << "Time: " << timer.Get() << ", RPM: " << GetRPM() << endl;

	    //Drive ---------------------------------------------------------------------
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
        
        //Joystick 1 ----------------------------------------------------------------
        /*
        //position
        RUN_ONCE(joystick1, PERFORM_BALANCE_TRICK)
        {
            PerformBalanceTrick(joystick1);
        }
        
        //speed
        RUN_ONCE(joystick1, PERFORM_BALANCE_TRICK)
        {
            PerformBalanceTrickSpeed(joystick1);
        }
        */
/*
	    //target finder
        RUN_ONCE(joystick1, 2)
        {
            message("Finding targets");
	        distance = cyclops->GetDistanceToTarget();
	        angle = cyclops->GetAngleOffCenter();
            Rotate(angle);
        }
*/
        RUN_ONCE(joystick1, DRIVE_SET_BRAKES_ON)
        {
            SetBrakes(true);
            message("brakes set: on");
        }

        RUN_ONCE(joystick1, DRIVE_SET_BRAKES_OFF)
        {
            SetBrakes(false);
            message("brakes set: off");
        }

        RUN_ONCE(joystick1, GET_LEFT_ENCODER)
        {
            message("left encoder: %f", GetLeftEncoder());
        }

        RUN_ONCE(joystick1, GET_RIGHT_ENCODER)
        {
            message("right encoder: %f", GetRightEncoder());
        }

        //Shooter -------------------------------------------------------------------
        static float pvoltage = 0.0;
        static float max_speed = 4200;
        float percent = setspeed / max_speed;

        if(windup)
        {
            if(pvoltage < percent)
            {
                pvoltage+=0.01;
            }
            else
            {
                message("windup done");
                windup = false;
            }
        }
        else
        {
            if(pvoltage < percent)
            {
                pvoltage+=0.01;
            }
            else if(pvoltage > percent)
            {
                pvoltage-=0.01;
            }
        }

        SetShooterSpeed(pvoltage);

        //Joystick 2 ----------------------------------------------------------------
        RUN_ONCE(joystick2, SET_SPEED_FENDER_MEDIUM)
        {
            message("set speed fender medium");
            targetspeed = FENDER_MEDIUM; 
        }

        RUN_ONCE(joystick2, SET_SPEED_FENDER_HIGH)
        {
            message("set speed fender high");
            targetspeed = FENDER_HIGH;
        }

        RUN_ONCE(joystick2, SET_SPEED_KEY)
        {
            message("set speed key");
            targetspeed = KEY;
        }

        if(joystick2->GetRawButton(BRIDGE))
        {
            RunBridge(true); 
        }
        else if(!joystick2->GetRawButton(BRIDGE))
        {
            RunBridge(false);
        }
/*        
        RUN_ONCE(joystick1, 3)
        {
	       message("Starting balancing trick");
	       PerformBalanceTrick(joystick1);
        }
*/
        if(joystick2->GetRawButton(BALL_PICKUP))
	    {
	        m_pscBallPickup->Set(-1.0);
	    }
        else
        {
            m_pscBallPickup->Set(0.0);
        }

        if (joystick2->GetRawButton(BALL_FEEDER))
        {
            m_pscBallFeeder->Set(.55);
        }
        else if (joystick2->GetRawButton(BALL_FEEDER_UP))
        {
            m_pscBallFeeder->Set(-1.0);
        }
        else
        {
            m_pscBallFeeder->Set(0.0);
        }
 
        Wait(0.05);
    }

    //cyclops->Stop();
    //delete cyclops;
}

void MyRobot::RunBridge(bool up)
{
   if(up)
   {
       m_pscBridge->Set(-0.5);
   }
   else
   {
       m_pscBridge->Set(0.7);
   }
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

// * Negative degrees rotates left, positive degrees rotates right
void MyRobot::Rotate(float degrees)
{
    float tireCircumference;
    float robotCircleDiameter;
    float robotCircleCircumference;
    float distanceOfTravel;
    float rotations;
    float ticks;
    float ticks_right;
    float ticks_left;
    bool rotate = true;

    tireCircumference = 3.14 * TIRE_DIAMETER;
    robotCircleDiameter = DISTANCE_BETWEEN_WHEELS;
    robotCircleCircumference = 3.14 * robotCircleDiameter;
    distanceOfTravel = robotCircleCircumference / (360 / degrees);
    rotations = distanceOfTravel / TIRE_DIAMETER;
    ticks = rotations * 250;

    DriveControlMode(CANJaguar::kPosition);

    if(degrees > 0)
    {
        ticks_left = GetLeftEncoder() - ticks;
        ticks_right = GetRightEncoder() + ticks;
    }
    else if(degrees < 0)
    {
        ticks_left = GetLeftEncoder() + ticks;
        ticks_right = GetRightEncoder() - ticks;
    }

    while(rotate)
    {
        GetWatchdog().Feed();

        DriveControlPosition(ticks_right, ticks_left);
        
        if(joystick2->GetRawButton(SHOOTER_SHOOT))
        {
            rotate = false;
        }
    }
    
    DriveControlMode(CANJaguar::kPercentVbus);
}

void MyRobot::DriveControlPosition(float position_right, float position_left)
{
    m_pscLeft1->Set(position_left);
    
    m_pscRight1->Set(position_right);
}

void MyRobot::DriveControlSpeed(float speed_right, float speed_left)
{
    m_pscLeft1->Set(speed_left);
    
    m_pscRight1->Set(speed_right);
}

void MyRobot::SetShooterSpeed(float speed)
{
    m_pscShooterMaster->Set(speed);
    m_pscShooterSlave1->Set(speed);
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
    DriveControlMode(CANJaguar::kPosition);
    
    float initial_position_right = GetRightEncoder();
    float initial_position_left = GetLeftEncoder();
    float target_position_right = initial_position_right+750;
    float target_position_left = initial_position_left-750;

    while (IsOperatorControl() && IsEnabled())
    {
        GetWatchdog().Feed();

        DriveControlPosition(target_position_right, target_position_left);
        
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
            break;
        }
    }     
    
    DriveControlMode(CANJaguar::kPercentVbus);
}

void MyRobot::PerformBalanceTrickSpeed(MyJoystick *joy)
{
    DriveControlMode(CANJaguar::kSpeed);

    while (IsOperatorControl() && IsEnabled())
    {
        GetWatchdog().Feed();

        DriveControlSpeed(25,25);
        
        RUN_ONCE(joystick1, 8)
        {
            message("left rpm: %f", m_pscLeft1->GetSpeed());
        }

        RUN_ONCE(joystick1, 9)
        {
            message("right rpm: %f", m_pscRight1->GetSpeed());
        }

        RUN_ONCE(joystick1, 2)
        {
            break;
        }
    }     
    
    DriveControlMode(CANJaguar::kPercentVbus);
}

void MyRobot::DriveControlMode(CANJaguar::ControlMode control)
{
    float p, i, d;

    switch (control) {
    case CANJaguar::kPosition:
    	m_pscLeft1->ConfigEncoderCodesPerRev(1);
        m_pscLeft1->ChangeControlMode(CANJaguar::kPosition);
    	m_pscRight1->ConfigEncoderCodesPerRev(1);
        m_pscRight1->ChangeControlMode(CANJaguar::kPosition);
        p = -0.2;
        i = -0.00217;
        d = 0;

        m_pscLeft1->SetPID(p,i,d);
        m_pscRight1->SetPID(p,i,d);
        m_pscLeft1->EnableControl();
        m_pscRight1->EnableControl();
        message("drive position control enabled");
	    break;
    
    case CANJaguar::kSpeed:
    	m_pscLeft1->ConfigEncoderCodesPerRev(250);
    	m_pscLeft1->ChangeControlMode(CANJaguar::kSpeed);
    	m_pscRight1->ConfigEncoderCodesPerRev(250);
        m_pscRight1->ChangeControlMode(CANJaguar::kSpeed);
        p = 0.2;
        i = 0.0;
        d = 0.0;

        m_pscLeft1->SetPID(p,i,d);
        m_pscRight1->SetPID(p,i,d);
        m_pscLeft1->EnableControl();
        m_pscRight1->EnableControl();
        message("drive speed control enabled");
        break;

    case CANJaguar::kPercentVbus:
        m_pscLeft1->DisableControl();
        m_pscRight1->DisableControl();
    	m_pscLeft1->ConfigEncoderCodesPerRev(250);
        m_pscLeft1->ChangeControlMode(CANJaguar::kPercentVbus);
    	m_pscRight1->ConfigEncoderCodesPerRev(250);
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
