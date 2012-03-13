#include "iostream"
#include "fstream"
#include "WPILib.h"
#include "MyRobot.h"
#include "Vision/RGBImage.h"
#include "Vision/BinaryImage.h"
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
    GetWatchdog().SetEnabled(true);

    float p, i, d;
    p = 0.029;
    i = 0.0;
    d = 0.0;

    const float targetspeed = 3000.0;
    float setspeed = 2*targetspeed;
    float speed = 0.0;

    SetBrakes(false);
    
    ofstream myfile;
    myfile.open("example.txt");

    Timer timer;
    timer.Reset();
    timer.Start();

    AxisCamera &camera = AxisCamera::GetInstance("10.9.80.11");
    
    while (IsOperatorControl())
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
	if(throttle < 0.08 && throttle > -0.08)
	{
	    throttle = 0.0;
	}
	//set default fLeft and fRight to throttle
	float fLeft = throttle;
        float fRight = fLeft;

        //if statements for distributing power to left and right depending on gain value 
	if(gain>0.05 || gain<-0.05)
	{
	    message("throttle: %f", throttle);
	    fLeft = throttle+(gain);
	    fRight = throttle-(gain);
	}

        Drive(fLeft, fRight);

	//target finder
        RUN_ONCE(joystick1, 1)
        {
            message("finding targets");
            vector<vector<int> > points = GetTargetCenters(camera);
            
            for(unsigned i = 0; i < points.size(); i++)
            {
                int x = points.at(i).at(0);
                int horizontal = x-160;
                int width = points.at(i).at(2);
                float distance = GetDistanceToTarget(width);
                message("distance from target: %f", distance);
                if(horizontal > 0)
                {
                    message("you are %d pixels off to the left", horizontal);
                }
                else if(horizontal < 0)
                {
                    message("you are %d pixels off to the right", horizontal);
                }
                else 
                {
                    message("you are on target");
                }
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
	
        if(joystick1->GetRawButton(2))
        {
            if(myfile.is_open())
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

        /*
        if(joystick1->GetRawButton(5))
	    {
	        m_pscBallPickup->Set(1.0);
	    }
	    else if(joystick1->GetRawButton(4))
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
*/       
///////////////////this if for joystick2 button test/////////////////////

	RUN_ONCE(joystick2, 1)
	{
	    message("joystick2: Button 1");
	}
	
	RUN_ONCE(joystick2, 2)
	{
	    message("joystick2: Button 2");
	}
	
	RUN_ONCE(joystick2, 3)
	{
	    message("joystick2: Button 3");
	}
	
	RUN_ONCE(joystick2, 4)
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
    float voltage = m_pscShooterMaster->GetOutputVoltage();
    m_pscShooterSlave1->Set(voltage);
    m_pscShooterSlave2->Set(voltage);
    m_pscShooterSlave3->Set(voltage);
    m_pscShooterMaster->Set(speed);
}

float MyRobot::GetRightEncoder(void)
{
    return m_pscRight1->GetPosition();
}   

float MyRobot::GetLeftEncoder(void)
{
    return m_pscLeft1->GetPosition();
}

vector<vector<int> > MyRobot::GetTargetCenters(AxisCamera &camera)
{
    vector<vector<int> > points;
    if(camera.IsFreshImage())
    {
        Threshold threshold(92,139,76,255,90,255);
        ParticleFilterCriteria2 criteria[] = {
            {IMAQ_MT_BOUNDING_RECT_WIDTH, 20, 400, false, false},
            {IMAQ_MT_BOUNDING_RECT_HEIGHT, 18, 400, false, false}
        };
        ColorImage *image = camera.GetImage();
        BinaryImage *thresholdImage = image->ThresholdHSL(threshold);
        BinaryImage *bigObjectsImage = thresholdImage->RemoveSmallObjects(false, 1);
        BinaryImage *convexHullImage = bigObjectsImage->ConvexHull(false);
        BinaryImage *filteredImage = convexHullImage->ParticleFilter(criteria, 2);
        vector<ParticleAnalysisReport> *reports = filteredImage->GetOrderedParticleAnalysisReports();

        if(reports->size() == 0)
        {
            message("No targets");
        }
        else
        {
            for(unsigned i = 0; i < reports->size(); i++)
            {
                ParticleAnalysisReport *r = &(reports->at(i));
                vector<int> temp;
                temp.push_back(r->center_mass_x);
                temp.push_back(r->center_mass_y);
                temp.push_back(r->boundingRect.width);
                points.push_back(temp);
            }
        }
        delete filteredImage;
        delete bigObjectsImage;
        delete thresholdImage;
        delete image;
        delete reports;
    }
    else
    {
        message("No fresh image");
    }
    return points;
}

float MyRobot::GetDistanceToTarget(float width)
{
    float tft = 2.0;
    float FOVp = 320.0;
    float theta = 24.0;
    float FOVft = ((tft/width) * FOVp)/2.0;
    float distance = FOVft/tan((3.14159*theta/180.0));
    return distance;
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
START_ROBOT_CLASS(MyRobot)
