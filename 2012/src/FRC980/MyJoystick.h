#include "WPILib.h"

#ifndef _MYJOYSTICK_H_
#define _MYJOYSTICK_H_

class MyJoystick : public Joystick
{

    public:

	MyJoystick(UINT32 port);
	MyJoystick(UINT32 port, UINT32 numAxisTypes, UINT32 numButtonTypes);
	~MyJoystick();

	bool Dead(void);

};

#endif
