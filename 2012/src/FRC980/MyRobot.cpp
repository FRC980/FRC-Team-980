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
    float maxcurrent = 0;

    GetWatchdog().SetEnabled(true);
    GetWatchdog().Feed();

    while(IsOperatorControl())
    {
        float x, y;
        x = joystick1->GetX();
        y = joystick1->GetY();
        if(joystick1->GetTrigger())
        {
            Drive(y);
            if(joystick1->GetRawButton(3))
            {
                float current = jag1->GetOutputCurrent();
                message("Current: %f", current);
                if(current >= maxcurrent)
                {
                    maxcurrent = current;
                }
            }
        }
        else
        {
            Drive(0);
        }

        if(joystick1->GetRawButton(2))
        {
            message("Max Current: %f", maxcurrent);
        }
        
        GetWatchdog().Feed();
    }

}

void MyRobot::Drive(float speed)
{
    jag1->Set(speed);
}

START_ROBOT_CLASS(MyRobot);
