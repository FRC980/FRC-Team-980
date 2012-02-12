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

MyRobot::MyRobot(void)
{
    jag1 = new Jaguar(Jaguar1);
    SteeringWheel = new SteeringWheel(2)
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
    bool printMessage = false;

    GetWatchdog().SetEnabled(true);
    GetWatchdog().Feed();

    while(IsOperatorControl())
    {
        float x, y;
        x = SteeringWheel2->GetX();
        y = joystick1->GetY();
	RUN_ONCE(SteeringWheel2, 2)
        RUN_ONCE(joystick1, 1)
        {
            printf("\nTrigger pressed");
            printf("\nx: %f", x);
            printf("\ny: %f", y);
        }

        GetWatchdog().Feed();
    }

}

void MyRobot::Drive(float speed)
{
    jag1->Set(speed);
}

START_ROBOT_CLASS(MyRobottcd)
