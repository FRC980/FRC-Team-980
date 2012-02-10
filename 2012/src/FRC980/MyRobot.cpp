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

void message(char *fmt, ...)
{
    char message[256];

    va_list args;
    va_start(args, fmt);
    vsnprintf(message, 256, fmt, args);
    va_end(args);

    setErrorData(message, strlen(message), 100);
}

MyRobot::MyRobot(void)
{
    jag1 = new CANJaguar(1, CANJaguar::kSpeed);
    jag1->ConfigEncoderCodesPerRev(250);
    jag1->SetSpeedReference(CANJaguar::kSpeedRef_Encoder);
    jag1->ConfigMaxOutputVoltage(12);

    joystick1 = new Joystick(1);

    ds = DriverStation::GetInstance();
    GetWatchdog().SetExpiration(0.1);
}

MyRobot::~MyRobot(void)
{
    delete jag1;
    delete joystick1;
    delete ds;
}

void MyRobot::Autonomous(void)
{
}

void MyRobot::OperatorControl(void)
{
    GetWatchdog().SetEnabled(true);
    float p, i, d;
    p = 0.001;
    i = 0.9;
    d = 0.0;
    jag1->SetPID(p,i,d);
    jag1->EnableControl();
    while(IsOperatorControl())
    {
        bool recording = false;
        float y;
        y = joystick1->GetY();
        if(joystick1->GetRawButton(2))
        {
            jag1->Set(y*4500);
        }
        else if(joystick1->GetTrigger())
        {
            Drive(0.35);
        }
	else if(joystick1->GetRawButton(3))
	{
	    Drive(.5);	
	}
        else
        {
            Drive(0.0);
        }

        RUN_ONCE(joystick1, 4)
        {
            i+=0.01;
            jag1->DisableControl();
            jag1->SetPID(p,i,d);
            jag1->EnableControl();
            printf("i: %f", i);
        }

        RUN_ONCE(joystick1, 5)
        {
            p+=0.0001;
            jag1->DisableControl();
            jag1->SetPID(p,i,d);
            jag1->EnableControl();
            printf("p: %f", p);
        }
        GetWatchdog().Feed();
    }
}

void MyRobot::Drive(float speed)
{
    jag1->Set(speed);
}

START_ROBOT_CLASS(MyRobot);
