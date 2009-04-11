/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef __DRIVER_STATION_H__
#define __DRIVER_STATION_H__

#include "Dashboard.h"
#include "SensorBase.h"
#include "Task.h"

struct FRCControlData;
class AnalogChannel;

/**
 * Provide access to the network communication data to / from the Driver
 * Station.
 */
class DriverStation : public SensorBase
{
public:
	enum Alliance {kRed, kBlue, kInvalid};

	virtual ~DriverStation();
	static DriverStation *GetInstance();

	static const UINT32 kBatterySlot = 1;
	static const UINT32 kBatteryChannel = 8;
	static const UINT32 kJoystickPorts = 4;
	static const UINT32 kJoystickAxes = 6;

	float GetStickAxis(UINT32 stick, UINT32 axis);
	short GetStickButtons(UINT32 stick);

	float GetAnalogIn(UINT32 channel);
	bool GetDigitalIn(UINT32 channel);
	void SetDigitalOut(UINT32 channel, bool value);
	bool GetDigitalOut(UINT32 channel);

	bool IsDisabled();
	bool IsAutonomous();
	bool IsOperatorControl();

	UINT32 GetPacketNumber();
	Alliance GetAlliance();
	UINT32 GetLocation();

	float GetBatteryVoltage();

	Dashboard& GetDashboardPacker(void) {return m_dashboard;}

protected:
	DriverStation();

	void GetData();
	void SetData();

private:
	static void InitTask(DriverStation *ds);
	static DriverStation *m_instance;
	///< TODO: Get rid of this and use the semaphore signaling
	static const float kUpdatePeriod = 0.02;

	void Run();

	struct FRCControlData *m_controlData;
	char *m_userControl;
	char *m_userStatus;
	UINT8 m_digitalOut;
	AnalogChannel *m_batteryChannel;
	Task m_task;
	Dashboard m_dashboard;
};

#endif

