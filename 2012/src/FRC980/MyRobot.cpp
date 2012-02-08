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
    jag1 = new CANJaguar(1);
    joystick1 = new Joystick(1);
    jag1->ConfigEncoderCodesPerRev(250);
    jag1->SetPositionReference(CANJaguar::kPosRef_QuadEncoder);
    jag1->ConfigMaxOutputVoltage(12);
    jag1->ConfigNeutralMode(CANJaguar::kNeutralMode_Coast);

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

    while(IsOperatorControl())
    {
        float y;
        if(joystick1->GetTrigger())
        {
            Drive(1.0);
        }
	else if(joystick1->GetRawButton(3))
	{
	    Drive(.5);	
	}
        else
        {
            Drive(0);
        }

        if(joystick1->GetRawButton(2))
        {
            message("Encoder: %f", jag1->GetPosition());
        }

        GetWatchdog().Feed();
    }

}

void MyRobot::Drive(float speed)
{
    jag1->Set(speed);
}

START_ROBOT_CLASS(MyRobot);
