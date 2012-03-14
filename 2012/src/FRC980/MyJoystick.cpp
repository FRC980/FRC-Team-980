#include "MyJoystick.h"

MyJoystick::MyJoystick(UINT32 port) : Joystick(port)
{
}

MyJoystick::MyJoystick(UINT32 port, UINT32 numAxisTypes, 
                       UINT32 numButtonTypes) : 
		Joystick(port, numAxisTypes, numButtonTypes)
{
}

MyJoystick::~MyJoystick()
{
}

bool MyJoystick::Dead(void)
{
    if ((GetX() == 0) && (GetY() == 0))
    {
	return true;
    } 
    else
    {
	return false;
    }
}
