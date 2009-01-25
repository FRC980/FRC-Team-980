/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.							  */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in $(WIND_BASE)/WPILib.  */
/*----------------------------------------------------------------------------*/

#ifndef I2C_H
#define I2C_H

#include "SensorBase.h"

class DigitalModule;

/**
 * I2C bus interface class.
 * 
 * This class is intended to be used by sensor (and other I2C device) drivers.
 * It probably should not be used directly.
 * 
 * It is constructed by calling DigitalModule::GetI2C() on a DigitalModule object.
 */
class I2C : SensorBase
{
	friend class DigitalModule;
public:
	virtual ~I2C();
	void Write(UINT8 registerAddress, UINT8 data);
	void Read(UINT8 registerAddress, UINT8 count, UINT8 *data);
	void Broadcast(UINT8 registerAddress, UINT8 data);

	bool VerifySensor(UINT8 registerAddress, UINT8 count, const UINT8 *expected);
private:
	I2C(DigitalModule *module, UINT8 deviceAddress);

	DigitalModule *m_module;
	UINT8 m_deviceAddress;
};

#endif

